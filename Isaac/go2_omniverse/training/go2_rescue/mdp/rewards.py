# Speed-scaled wrappers around Spot's gait and air-time rewards.
#
# THE PROBLEM THESE SOLVE, observed on the first trained policy: it trots beautifully at
# speed, and at a walk or a standstill it stamps on the spot, lifting its feet as high and
# as long as it does at full speed. It looks like it is jumping between two diagonal pairs.
#
# That is not a training failure. It is the arithmetic of Spot's two largest positive
# terms, which between them are 15 of the ~25 points of positive weight in the set:
#
#   GaitReward (10.0) ends with
#       torch.where(cmd > 0.0 or body_vel > velocity_threshold, sync * async, 0.0)
#   air_time_reward (5.0) with
#       torch.where(cmd > 0.0 or ..., where(t_max < mode_time, t_min, 0), stance_reward)
#
# Both gate on `cmd > 0.0` — a BINARY test on the command's magnitude. A command of
# 0.01 m/s pays exactly the same gait reward as one of 1.0 m/s. And `air_time_reward` pays
# for air time up to a CONSTANT `mode_time` however slowly the robot is asked to move, so
# the cadence it learns is speed-independent by construction.
#
# The one thing that would have taught it to stand still is `rel_standing_envs`, which the
# stock velocity env sets to 0.02: two environments in a hundred are ever commanded to
# stand, and everything else is asked to move at least a little, and is paid the full 15
# points for trotting whether or not that is a sensible way to satisfy the request.
#
# So: multiply both by a ramp on the commanded speed. Below `full_speed` the gait is worth
# proportionally less than standing quietly, and at zero command it is worth nothing — at
# which point `joint_position_penalty`'s `stand_still_scale` (5x) is the dominant term and
# the robot is paid to hold its default pose instead.
#
# WHY A WRAPPER RATHER THAN A COPY of Spot's functions: the originals stay the upstream
# ones, so a fix or a change there is inherited rather than silently diverged from. The
# only thing added here is the multiplier.

from __future__ import annotations

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import ManagerTermBase
from isaaclab.sensors import ContactSensor

import isaaclab_tasks.manager_based.locomotion.velocity.config.spot.mdp as spot_mdp


def command_speed_scale(env, command_name: str, full_speed: float) -> torch.Tensor:
    """Linear ramp on the commanded speed, saturating at 1.0.

    The norm is taken over ALL THREE command components, yaw included, which is what makes
    an in-place turn count as motion. Nav2's settle commands exactly that — zero linear
    velocity with a yaw rate near 0.93 rad/s — and a scale computed from the linear terms
    alone would rate that as standing still and switch the gait reward off during the one
    manoeuvre this robot most needs a stable trot for.
    """
    cmd = torch.linalg.norm(env.command_manager.get_command(command_name), dim=1)
    return torch.clamp(cmd / full_speed, max=1.0)


def air_time_reward_scaled(
    env,
    asset_cfg,
    sensor_cfg,
    mode_time: float,
    velocity_threshold: float,
    command_name: str = "base_velocity",
    full_speed: float = 0.5,
) -> torch.Tensor:
    """Spot's air-time reward, scaled down as the commanded speed approaches zero."""
    reward = spot_mdp.air_time_reward(env, asset_cfg, sensor_cfg, mode_time, velocity_threshold)
    return reward * command_speed_scale(env, command_name, full_speed)


class GaitRewardScaled(spot_mdp.GaitReward):
    """Spot's gait reward, scaled down as the commanded speed approaches zero.

    Subclassed rather than wrapped in a function because the original is a
    :class:`ManagerTermBase` — it resolves the synchronised foot pairs once in ``__init__``
    against the contact sensor, and a plain function could not hold that state. The extra
    ``command_name`` / ``full_speed`` entries in ``params`` are ignored by the parent's
    ``__init__`` (it reads only the keys it knows) and arrive here as keyword arguments.
    """

    def __call__(
        self,
        env,
        std: float,
        max_err: float,
        velocity_threshold: float,
        synced_feet_pair_names,
        asset_cfg,
        sensor_cfg,
        command_name: str = "base_velocity",
        full_speed: float = 0.5,
    ) -> torch.Tensor:
        reward = super().__call__(
            env, std, max_err, velocity_threshold, synced_feet_pair_names, asset_cfg, sensor_cfg
        )
        return reward * command_speed_scale(env, command_name, full_speed)


def terrain_height(env, sensor_cfg) -> torch.Tensor:
    """Mean ground height under the base, from the height scanner, as (N,).

    GUARDS AGAINST NON-FINITE HITS, which Isaac Lab's own terrain-relative terms do not. A
    ray that escapes the terrain mesh returns inf, one inf poisons a plain `torch.mean`, and
    a single NaN reward propagates into the policy and destroys the run. `base_height_l2`
    upstream has exactly this hole; four lines is cheap against a wrecked two-hour run.

    The scanner is centred on the base and spans 1.6 x 1.0 m, so this is a LOCAL ground
    datum, not a per-foot one -- good enough at Go2 scale, where the feet are well inside
    that patch, and it costs no extra raycasts because the scan is already an observation.
    """
    sensor = env.scene[sensor_cfg.name]
    hits = sensor.data.ray_hits_w[..., 2]
    finite = torch.isfinite(hits)
    safe = torch.where(finite, hits, torch.zeros_like(hits))
    return safe.sum(dim=1) / finite.sum(dim=1).clamp(min=1)


def foot_terrain_height(env, sensor_cfg, asset_cfg) -> torch.Tensor:
    """Ground height under EACH FOOT separately, as (N, F).

    WHY THE PATCH MEAN IS NOT GOOD ENOUGH, and it took a slope to show it. `terrain_height`
    averages the whole 1.6 x 1.0 m scan and hands one number back for all four feet. On flat
    ground that is exact. On a slope it is wrong by each foot's distance from the patch
    centre, and the Go2's front and rear feet are 0.387 m apart:

        slope     front/rear ground offset     as a fraction of the 0.08 m clearance target
          3 deg            1.0 cm                            13 %
         10 deg            3.4 cm                            43 %
         15 deg            5.2 cm                            65 %
         20 deg            7.0 cm                            88 %

    The SIGN is what does the damage. Going uphill the front feet stand on ground above the
    datum, so the reward scores them as already clear and stops asking them to lift -- they
    scuff. The rear feet stand below it, score as too low, and are pushed to over-lift --
    the robot bounces. Both symptoms were reported from a sim with real topology, on a policy
    whose terrain curriculum had only reached 3.3 deg of slope.

    NEAREST RAY, NOT AN INTERPOLATION. The scan is a 0.1 m grid, so the nearest hit is at
    most 0.05 m away along the slope direction -- 1.8 cm of height error at 20 deg against
    the 7.0 cm the patch mean carries, and unlike a plane fit it does not smooth a step edge
    into a ramp, which is the other half of what this terrain is made of. It costs no extra
    raycasts: the scan is already an observation.
    """
    sensor = env.scene[sensor_cfg.name]
    asset = env.scene[asset_cfg.name]
    hits = sensor.data.ray_hits_w                                    # (N, R, 3)
    feet = asset.data.body_pos_w[:, asset_cfg.body_ids, :2]          # (N, F, 2)

    # Same non-finite guard as `terrain_height`: a ray that escapes the mesh returns inf, and
    # one inf reaching the reward is a NaN that propagates into the policy. Here it has to be
    # excluded from the SEARCH as well, or a foot could select an infinite hit as its nearest.
    finite = torch.isfinite(hits).all(dim=-1)                        # (N, R)
    dist = torch.cdist(feet, hits[..., :2])                          # (N, F, R)
    dist = torch.where(finite.unsqueeze(1), dist, torch.full_like(dist, float("inf")))
    nearest = dist.argmin(dim=-1)                                    # (N, F)
    ground = torch.gather(torch.nan_to_num(hits[..., 2]), 1, nearest)

    # If an environment had no finite ray at all, every distance was inf and `argmin` picked
    # index 0 arbitrarily. Fall back to the patch mean, which carries its own guard.
    any_finite = finite.any(dim=1, keepdim=True)
    return torch.where(any_finite, ground, terrain_height(env, sensor_cfg).unsqueeze(1))


def foot_clearance_reward_terrain(
    env,
    asset_cfg,
    sensor_cfg,
    target_height: float,
    std: float,
    tanh_mult: float,
) -> torch.Tensor:
    """Spot's foot-clearance reward, measured against the TERRAIN instead of the world.

    WHY THIS EXISTS RATHER THAN spot_mdp.foot_clearance_reward. The original scores
    ``body_pos_w[..., 2]`` — the foot's absolute world height — against a fixed target. On a
    plane that is the height above ground; on the generated rough terrain it is not, and the
    term would reward clearance over a hill and punish it in a trough. That is why it was
    left out of the rough set entirely, and leaving it out is why nothing bounded swing
    height: ``air_time_reward`` pays for staying airborne and no term priced how HIGH.

    The fix is Isaac Lab's own, borrowed from ``base_height_l2``, which solves exactly this
    problem for the body::

        adjusted_target_height = target_height + mean(sensor.data.ray_hits_w[..., 2])

    i.e. offset the target by the terrain height the height scanner is already measuring.
    The scanner is centred on the base and spans 1.6 x 1.0 m, so its mean is a local ground
    datum rather than a per-foot one — good enough at Go2 scale, where the feet are well
    inside that patch, and it costs no extra raycasts because the scan is already in the
    observation.

    The ground datum comes from :func:`terrain_height`, which is where the non-finite guard
    lives -- see its docstring.

    KNOWN HOLE, INHERITED FROM UPSTREAM AND NOT FIXED HERE: the height error is multiplied by
    `tanh(tanh_mult * |foot horizontal velocity|)`. The intent is to score only SWINGING feet,
    since a planted one should not be asked to clear anything. The consequence is that a foot
    which does not move contributes ~0 to the error sum, so the term returns ~exp(0) = 1 --
    FULL MARKS FOR NEVER LIFTING. It is blind in exactly the direction that matters.

    Measured across two policies with completely different gaits -- a 2 Hz trot lifting 10 cm
    and a 7 Hz shuffle lifting 1.4 cm -- this term scored 0.4234 and 0.4192. It cannot tell
    them apart, and nothing else in the set prices swing height, which is why the reward set
    has no defence against a skating gait (see the entropy_coef note in
    `agents/rsl_rl_ppo_cfg.py`, which is currently the only thing preventing one).

    THE FIX, when someone takes it on, is to gate on CONTACT rather than on speed: pass the
    contact sensor, build a swing mask from `net_forces_w`, and score height error on feet
    that are airborne regardless of how fast they are travelling. Then a foot hovering a
    centimetre up is penalised instead of ignored. It is left undone here only because it
    changes what the term rewards and so needs its own training run to validate, and the
    weight would almost certainly have to rise with it.
    """
    asset = env.scene[asset_cfg.name]
    ground = terrain_height(env, sensor_cfg).unsqueeze(1)

    foot_height = asset.data.body_pos_w[:, asset_cfg.body_ids, 2] - ground
    foot_z_target_error = torch.square(foot_height - target_height)
    # Only SWINGING feet are scored: a planted foot has near-zero horizontal velocity, so
    # tanh sends its contribution to zero. Without this the term would fight the stance.
    foot_velocity_tanh = torch.tanh(
        tanh_mult * torch.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2)
    )
    reward = foot_z_target_error * foot_velocity_tanh
    return torch.exp(-torch.sum(reward, dim=1) / std)


def hip_abduction_penalty(env, asset_cfg, target: float = 0.0) -> torch.Tensor:
    """Penalise hip abduction away from an ABSOLUTE target, not away from the default pose.

    WHY NOT `joint_deviation_l1`. That term measures against `default_joint_pos`, and
    UNITREE_GO2_CFG's default pose abducts every hip by 0.1 rad (5.7 deg). So the stock term
    does not hold the legs vertical -- it holds them 5.7 deg splayed, and defends that splay
    against anything trying to straighten it. Measured on the first policy: the hips rest at
    4.60 deg, INSIDE the 5.73 deg the pose asks for, so the visible lean at a standstill is
    the target, not a tracking failure. The only way to straighten it is to move the target.

    `target` is a scalar and that works because the wanted value is ZERO, which is signless.
    The Go2's hip defaults are +0.1 on the left and -0.1 on the right (both outward), so any
    non-zero target would have to be applied per-leg with the abduction sign.

    L1, not L2, on purpose: the penalty stays linear in the error rather than collapsing near
    zero, so it keeps pulling the last few degrees in. It also prices a large excursion in
    proportion to its size, which is what acts on the swing -- a stride that flings the foot
    out to 35 deg costs five times one that stays inside 7.

    THE TENSION TO WATCH IS STRAFING. A quadruped crab-walks by abducting, and the command
    range includes lin_vel_y in (-1, 1), so this term is directly opposed to sideways
    tracking. The arithmetic says tracking wins where it matters: 0.1 rad of extra abduction
    on four hips costs 0.4 * weight, against a linear-velocity reward worth up to 5.0 with an
    absolute-error kernel that keeps its gradient. If a retrain comes back unable to strafe,
    this weight is the cause and the first to back off.
    """
    asset = env.scene[asset_cfg.name]
    return torch.sum(torch.abs(asset.data.joint_pos[:, asset_cfg.joint_ids] - target), dim=1)


def base_height_reward_terrain(env, asset_cfg, sensor_cfg, target_height: float, std: float) -> torch.Tensor:
    """Reward holding the body at a fixed height above the TERRAIN.

    WHAT THIS FIXES. Spot's reward set has no height term at all, and nothing else in it
    prices ride height: `joint_position_penalty` pulls the whole pose toward default, but it
    is one L2 norm over every joint, so the two joints that set height are averaged in with
    ten that do not. Measured on the first policy: 32.94 cm standing against 34.86-35.79 cm
    walking, i.e. the body rises 2-3 cm the moment it starts moving and sinks again when it
    stops. That reads as the robot sagging at a standstill; the measurement says the opposite
    -- 32.94 cm IS the default pose's own height, and it is the WALK that is off-nominal.

    So the target is the default pose's height, which makes this term agree with
    `joint_position_penalty` instead of fighting it: one posture, defended standing and
    moving. The alternative, targeting the walking height, would have had two terms pulling
    the standstill in opposite directions.

    EXPONENTIAL, NOT L2, to stay on the scale of the rest of the set. `base_height_l2`
    returns squared metres -- a 2.5 cm error is 6.25e-4, which needs a weight in the hundreds
    before it means anything next to a gait term of 10. `exp(-|err|/std)` is bounded in
    [0, 1] like Spot's velocity rewards, so its weight reads directly as its importance.

    The saturation is deliberate too: at std 0.02 a 10 cm error scores 0.007 and the gradient
    is gone, so on rough ground a robot that must genuinely duck or climb is not fought --
    the term shapes the nominal ride height and abandons the argument in terrain that has
    its own opinion.
    """
    asset = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2] - terrain_height(env, sensor_cfg)
    return torch.exp(-torch.abs(height - target_height) / std)


def joint_position_penalty_scoped(env, asset_cfg, stand_still_scale: float, velocity_threshold: float) -> torch.Tensor:
    """Spot's joint-position penalty, but actually restricted to the joints it is given.

    WHY THIS EXISTS, and it is not a preference. `spot_mdp.joint_position_penalty` reads::

        reward = torch.linalg.norm((asset.data.joint_pos - asset.data.default_joint_pos), dim=1)

    -- no `[:, asset_cfg.joint_ids]` anywhere. `asset_cfg` is used ONLY to look up the asset,
    so the term silently norms over all twelve joints however it is configured. The same is
    true of Spot's velocity, acceleration and torque penalties. Naming joints in their
    `SceneEntityCfg` does nothing at all, and reads in the config as though it does.

    That is not cosmetic here. `hip_deviation` holds the hips at ZERO abduction; the default
    pose abducts them by 0.1 rad; so as long as this term includes the hips it is pulling
    them back out, and the two terms settle at an equilibrium neither asked for. The
    first policy trained with them fighting still came out at 1.94 deg of resting splay,
    which says `hip_deviation` was winning -- but it was winning against an opponent that
    was never meant to be in the ring.

    Everything else is Spot's, including the `cmd > 0.0` binary gate on the stand-still
    scaling, which is left alone deliberately: unlike the gait and air-time rewards this term
    wants to be FULLY on at a standstill, so the gate's lack of a ramp is correct here.
    """
    asset = env.scene[asset_cfg.name]
    cmd = torch.linalg.norm(env.command_manager.get_command("base_velocity"), dim=1)
    body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    reward = torch.linalg.norm(
        asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids],
        dim=1,
    )
    return torch.where(torch.logical_or(cmd > 0.0, body_vel > velocity_threshold), reward, stand_still_scale * reward)


def foot_clearance_reward_arc(
    env,
    asset_cfg,
    sensor_cfg,
    contact_cfg,
    target_height: float,
    swing_time: float,
    std: float,
) -> torch.Tensor:
    """Reward a foot for tracing an ARC through its swing, not for sitting at an altitude.

    WHAT IT REPLACES AND WHY. `foot_clearance_reward_terrain` scores the foot against a
    CONSTANT target height for as long as it is moving horizontally. Read as an optimisation
    problem, that asks for: reach 8 cm as fast as possible, hold 8 cm for the whole traverse,
    return as fast as possible. The trained policy did precisely that -- measured over 2588
    swings at 0.5 m/s::

        height (cm)   2.2  4.7  6.5  7.6  8.3  8.7  8.9  8.8  8.4  7.6  5.9  2.8
        phase        0.00 0.09 0.18 0.27 0.36 0.45 0.55 0.64 0.73 0.82 0.91 1.00

    58% of the swing above 80% of peak height (a sine spends 41%) and a peak-to-mean vertical
    speed ratio of 2.63 (a sine, 1.60), with the last 18% of the swing dropping 3.1 cm --
    about 1.9 m/s straight down. The leg snaps up, hangs, and races back to the floor. That
    is not a training failure; it is the term's maximum.

    So the target MOVES with the swing: `target_height * sin(pi * phase)`, zero at lift-off,
    peak at mid-swing, zero at touchdown. Holding altitude is now penalised at both ends of
    the swing rather than rewarded throughout, and the reward's optimum is the arc itself.

    PHASE COMES FROM THE CONTACT SENSOR'S `current_air_time` over a CONSTANT `swing_time`,
    which should track `air_time.mode_time` (0.25 s) -- the period the air-time reward is
    already steering the gait toward. Normalising by the foot's own last swing instead is the
    better model on paper and was measurably worse in practice; the note in the body says
    why. A swing that overruns clips at phase 1 and is asked to come down.

    AND THE SWING GATE IS NOW CONTACT, NOT SPEED. The old term multiplied the height error by
    `tanh(k * |foot horizontal velocity|)`, so a foot that did not move contributed nothing
    and the term returned full marks for never lifting -- it scored 0.4234 for a 10 cm trot
    and 0.4192 for a 1.4 cm shuffle, blind to the difference. Gating on `current_air_time > 0`
    means an airborne foot is scored on its height whatever its horizontal speed, which is
    the property that was missing.
    """
    asset = env.scene[asset_cfg.name]
    contacts = env.scene.sensors[contact_cfg.name]

    air_time = contacts.data.current_air_time[:, contact_cfg.body_ids]
    swinging = air_time > 0.0

    # PHASE IS NORMALISED BY A CONSTANT `swing_time`, NOT BY THE FOOT'S OWN LAST AIR TIME,
    # and that is the result of an experiment rather than laziness. Using `last_air_time` is
    # the better model on paper -- a gait is periodic, so the previous swing predicts this
    # one, and measured swings run 196-290 ms against a constant 250. It also removes the
    # only thing penalising a SHORTER swing: with a constant denominator a rushed swing ends
    # at a phase below 1 where the target is still high, which costs; with a self-normalising
    # one it does not, and the policy promptly raised its stride from 2.4 Hz to 3.1 and
    # lowered the arc to make the faster swing cheaper. Worse on both counts.
    phase = torch.clamp(air_time / swing_time, max=1.0)
    target = target_height * torch.sin(torch.pi * phase)

    height = asset.data.body_pos_w[:, asset_cfg.body_ids, 2] - foot_terrain_height(env, sensor_cfg, asset_cfg)

    # SQUARED ERROR, SUMMED, `std` IN SQUARE METRES -- AND THIS IS A DELIBERATELY GENTLE
    # TERM. Two attempts were made to give it more authority over the swing and both made
    # the gait worse:
    #
    #   kernel / std / weight            plateau (0.5, 1.0, turn)   peak height
    #   squared, 0.02 m^2, 1.0  <- here   58 / 50 / 58 %            6.8 / 8.5 / 8.7 cm
    #   absolute, 0.03 m,  2.0            58 / 67 / 75 %            4.4 / 4.9 / 5.0 cm
    #   squared,  0.0012 m^2, 2.5         67 / 67 / 75 %            4.2 / 4.6 / 4.5 cm
    #
    # The absolute kernel fails for a reason worth knowing -- L1 is minimised by the MEDIAN,
    # so a flat line is a good L1 fit to a sine -- but the squared version at a tight `std`
    # failed the same way, which rules that out as the explanation. The real one is in the
    # reward breakdown: at weight 2.5 the policy tracked the arc LESS well than at 1.0,
    # because flying low and flat is worth about 0.85 reward units across
    # `action_smoothness`, `joint_pos`, `hip_deviation`, `gait` and the base terms, against
    # the ~0.72 this term can charge for abandoning the arc. EVERY OTHER TERM IN THE SET
    # PENALISES VERTICAL MOTION and this is the only one asking for it, so the flat
    # compromise sits near the target's mean height -- 8 * 2/pi = 5.1 cm -- which is within a
    # centimetre of where all three attempts landed.
    #
    # Pushing past that needs roughly another 3x, at which point this out-weighs the gait
    # term. The swing SHAPE is not reachable by scalar reward shaping in this set; it needs a
    # reference trajectory or a motion prior. What this term IS worth keeping for is turn
    # regularity, which it improved from 23.0% stride-period variation to 8.5% -- the arc
    # target and the contact-based swing gate both earn their place there.
    error = torch.square(height - target) * swinging
    return torch.exp(-torch.sum(error, dim=1) / std)


def gait_pair_duty_penalty(env, pair_a_cfg, pair_b_cfg, command_name: str = "base_velocity",
                           full_speed: float = 0.5) -> torch.Tensor:
    """Penalise the two diagonal pairs spending different FRACTIONS of the stride on the ground.

    THE HOLE THIS FILLS. `GaitReward` is the largest positive term in the set (weight 10.0)
    and everything it grades is a PHASE relationship: feet in step within a diagonal pair,
    pairs in anti-phase with each other. A limp satisfies all of it. Measured on
    2026-09-12_11-46-13 turning in place at 1.0 rad/s:

        trot phase    FL 0.00  FR 0.50  RL 0.49  RR 0.98      <- textbook trot
        per foot      FL 67%   FR 37%   RL 35%   RR 65%       <- 1.84x duty imbalance

    Nothing else in the set asks the pairs to carry the robot for equal fractions of the
    stride, so the policy is free to stand on FL+RR for two thirds of every cycle and touch
    FR+RL down briefly, which is what a limp is and what it looks like.

    IT MUST BE A FRACTION, NOT A DURATION, AND THIS IS NOT A DETAIL -- the first version of
    this term was `|stance_a - stance_b|` IN SECONDS and it destroyed the gait. The cheapest
    way to make a difference of durations small is to make every duration small, so instead
    of balancing the stride the policy collapsed it (run 2026-09-12_13-26-03, against v1):

                             v1        v2 with the seconds version
        stride, 0.5 m/s    2.50 Hz     6.70 Hz
        foot lift          6.4 cm      2.3 cm
        duty               62.6 %      80.0 %
        trot phase    0.00/.50/.49/.98  0.00/.14/.30/.94
        swing segments     1872        3

    It reached a duty ratio of 1.08x by skating -- the stated goal, met the wrong way, and
    2.3 cm of clearance is useless on rough ground. `Episode_Reward/gait_pair_duty` never
    decayed over 2000 iterations, which is the tell: a penalty the policy has actually
    satisfied shrinks toward zero, and one it has found a side door around does not.

    Duty FRACTION is dimensionless, so shortening the cycle gains exactly nothing and the
    only way left to reduce it is the intended one.

    SCALED BY COMMANDED SPEED, like the gait and air-time rewards: `last_contact_time` and
    `last_air_time` hold the last COMPLETED phase, so a robot standing still carries whatever
    the values were when it stopped. Ungated, a policy that had just finished an uneven turn
    would carry a standing penalty it could only clear by taking more steps.
    """
    contacts: ContactSensor = env.scene.sensors[pair_a_cfg.name]
    stance = contacts.data.last_contact_time
    swing = contacts.data.last_air_time
    # Their sum is one stride for that foot, so this is its duty fraction in [0, 1]. The
    # clamp covers a foot that has not yet completed a phase, which reads as duty 0 for
    # both pairs and so contributes nothing rather than a spurious imbalance.
    duty = stance / torch.clamp(stance + swing, min=1e-3)
    # TWO SEPARATE CFGS RATHER THAN ONE OF FOUR BODIES, because `find_bodies` returns indices
    # in the articulation's own order, not in the order the names were listed -- so indexing
    # [0],[3] out of a single four-body cfg to get a diagonal pair is only right by luck.
    a = duty[:, pair_a_cfg.body_ids].mean(dim=1)
    b = duty[:, pair_b_cfg.body_ids].mean(dim=1)
    return torch.abs(a - b) * command_speed_scale(env, command_name, full_speed)


# ---------------------------------------------------------------------------------------
# Rescue additions. Everything above is P2Dingo's, copied 2026-09-12; what follows exists
# for the stairs in Rescue's maze and is not in the P2Dingo set.
# ---------------------------------------------------------------------------------------


def foot_stumble_penalty(env, sensor_cfg, ratio: float = 4.0, min_force: float = 1.0) -> torch.Tensor:
    """Count feet whose contact force is mostly HORIZONTAL -- a toe catching a riser.

    WHAT IT CATCHES. On a stair the characteristic failure is not a fall, it is the swing
    foot arriving a centimetre too low and hitting the vertical face of the next step. The
    contact sensor sees that as a force that is nearly all horizontal, where a normal
    footfall is nearly all vertical and a push-off is at most friction-limited, i.e. under
    1x the vertical load. Nothing else in the set can see this: `foot_clearance` scores
    height against the ground under the foot, and a foot that has just hit a riser is at a
    perfectly good height above the tread it is standing on.

    WHY A RATIO AND NOT A THRESHOLD. Contact force scales with how hard the robot is
    walking, so an absolute horizontal-force limit would fire on every fast push-off and
    stay silent on a gentle scuff. `ratio` compares the two components of the SAME contact,
    which is dimensionless: with a friction coefficient of 1.0 in training, a planted foot
    cannot physically exceed 1x, so 4x is far outside anything a legitimate step produces
    and well inside what a riser strike produces (vertical ~0). `min_force` stops the
    sensor's noise floor from tripping it when a foot is barely touching.

    SIZING IS A FIRST SETTING, NOT A MEASUREMENT. This returns the NUMBER of stumbling feet
    per step, so at weight -0.5 a single foot dragging along a riser for a whole 0.25 s
    swing (12 policy steps) costs 6.0 -- roughly what the gait term pays per step, so the
    stumble is worth about one step of gait. That is the intent: a stumble should cost about
    as much as the step it ruins. It is legged_gym's `feet_stumble` with the ratio dropped
    from 5 to 4 for a lighter robot. If the terrain curriculum stalls at the stair levels
    with `Episode_Reward/foot_stumble` flat, this is the first weight to raise.
    """
    contacts: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = contacts.data.net_forces_w[:, sensor_cfg.body_ids, :]            # (N, F, 3)
    lateral = torch.linalg.norm(forces[..., :2], dim=-1)
    vertical = torch.abs(forces[..., 2])
    stumble = (lateral > ratio * vertical) & (lateral > min_force)
    return stumble.float().sum(dim=1)


def _scoped_norm(env, asset_cfg, attr: str) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    return torch.linalg.norm(getattr(asset.data, attr)[:, asset_cfg.joint_ids], dim=1)


def joint_torques_penalty_scoped(env, asset_cfg) -> torch.Tensor:
    """Spot's `joint_torques_penalty`, honouring `asset_cfg.joint_ids`.

    WHY THESE THREE EXIST. Spot's `joint_torques_penalty`, `joint_acceleration_penalty` and
    `joint_velocity_penalty` take an `asset_cfg` and then norm over `asset.data.*` WHOLE --
    the joint ids are resolved and never read (P2Dingo's config notes this and spells the
    scope `.*` to be honest about it). On the bare Go2 that is harmless: `.*` and the legs
    are the same twelve joints. On the welded robot they are not. The arm's six joints hold
    their fold against gravity with a steady torque and pick up velocity and acceleration
    from every body motion, so the unscoped terms would charge the gait for the arm's
    servos -- a constant offset on torque, and on acceleration a second, unintended damping
    of body jerk routed through the arm. `mdp.joint_torques_l2` upstream honours ids and
    could have been used, but it is a SQUARED sum where Spot's is a norm, and the weights
    in this set were sized against the norm. Same maths, twelve columns.
    """
    return _scoped_norm(env, asset_cfg, "applied_torque")


def joint_acceleration_penalty_scoped(env, asset_cfg) -> torch.Tensor:
    """Spot's `joint_acceleration_penalty`, honouring `asset_cfg.joint_ids`. See above."""
    return _scoped_norm(env, asset_cfg, "joint_acc")


def joint_velocity_penalty_scoped(env, asset_cfg) -> torch.Tensor:
    """Spot's `joint_velocity_penalty`, honouring `asset_cfg.joint_ids`. See above."""
    return _scoped_norm(env, asset_cfg, "joint_vel")


# ----------------------------------------------------------------------------------------------
# EXPERIMENT TERMS (2026-09-14). Used only by the short fine-tune arms in ../experiments.py;
# the main task does not reference them. Each exists to test one hypothesis from
# training/measure_bench.py and is kept or deleted on that measurement.
# ----------------------------------------------------------------------------------------------


class FootPlacementPenalty(ManagerTermBase):
    """Where each foot LANDS fore-aft, against Raibert's neutral point. Gentle and speed-scaled.

    WHY. measure_bench.py on the 2026-09-13 policies, flat ground, 0.5 m/s: the front feet land
    10.4 cm BEHIND their thigh joints and the rear feet 17.8 cm AHEAD of theirs, so the two land
    19.7 cm apart where the hips are 38.7 cm apart -- both pairs gather under the body. P2Dingo's
    policy, same probe, lands within 1-2 cm of the neutral point on every foot. So the placement
    is learnable on this robot and something in the Rescue setup lost it.

    THE TARGET, per foot, in the yaw frame, relative to its own thigh joint (the leg's pitch
    axis):

        target_x = nominal_x + v_cmd_x * stance_time / 2

    `nominal_x` is where the default pose puts the foot (front -1.6 cm, rear -7.7 cm, measured),
    and the second term is Raibert's: a foot placed half a stance's travel ahead of the hip is
    centred under it at mid-stance. With a trot at f Hz the stance lasts 1/(2f), so the offset
    is v/(4f): 6.25 cm at 0.5 m/s and 2 Hz, 12.5 cm at 1.0 m/s. The COMMANDED velocity is used,
    not the measured one, so the term asks the leg to reach for the speed that was requested
    rather than to agree with a speed the robot has fallen short of.

    WHY AT TOUCHDOWN ONLY. Walk These Ways grades the foot continuously against a phase clock;
    this task has no clock and adding one would change the observation. Grading the one moment
    that decides placement needs no clock and cannot penalise the swing's path.

    Contact edges are taken from the foot force directly, not from the contact sensor's
    compute_first_contact(), which measure_bench.py found to miss ~15 % of touchdowns when read
    once per control step.

    GENTLE: a linear penalty in metres, clipped at `max_err`, scaled from zero at a standstill
    to full at `full_speed`, summed over the feet that touched down this step. Turning in place
    has no linear command and pays nothing.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._prev = torch.ones(env.num_envs, 4, dtype=torch.bool, device=env.device)
        self._nominal = torch.tensor(cfg.params["nominal_dx"], device=env.device)

    def reset(self, env_ids=None):
        if env_ids is None:
            self._prev[:] = True
        else:
            self._prev[env_ids] = True

    def __call__(self, env, asset_cfg, thigh_cfg, sensor_cfg, nominal_dx, stance_time: float = 0.25,
                 full_speed: float = 0.5, max_err: float = 0.20, force_threshold: float = 1.0):
        asset = env.scene[asset_cfg.name]
        contacts = env.scene.sensors[sensor_cfg.name]
        loaded = contacts.data.net_forces_w[:, sensor_cfg.body_ids, :].norm(dim=-1) > force_threshold
        touchdown = loaded & ~self._prev
        self._prev = loaded
        n = env.num_envs
        yq = math_utils.yaw_quat(asset.data.root_quat_w).repeat_interleave(4, 0)
        rel = asset.data.body_pos_w[:, asset_cfg.body_ids, :] - asset.data.body_pos_w[:, thigh_cfg.body_ids, :]
        dx = math_utils.quat_apply_inverse(yq, rel.reshape(-1, 3)).reshape(n, 4, 3)[..., 0]
        cmd = env.command_manager.get_command("base_velocity")
        target = self._nominal + cmd[:, 0:1] * (0.5 * stance_time)
        err = (dx - target).abs().clamp(max=max_err)
        scale = torch.clamp(torch.linalg.norm(cmd[:, :2], dim=1) / full_speed, max=1.0)
        return (err * touchdown.float()).sum(dim=1) * scale


def joint_position_penalty_thigh_relaxed(env, asset_cfg, stand_still_scale: float, velocity_threshold: float,
                                         moving_thigh_scale: float = 0.5):
    """`joint_position_penalty_scoped`, with the THIGH deviations scaled down while moving.

    The front thigh's pitch is what sets how far forward a front foot can reach, and the posture
    term pulls it toward the default pose on every step of a walk. This tests whether that pull
    is what shortens the reach. Standing is untouched: the scale applies only on the branch the
    original gives to a moving robot, and the calves keep their full weight throughout.
    """
    asset = env.scene[asset_cfg.name]
    cmd = torch.linalg.norm(env.command_manager.get_command("base_velocity"), dim=1)
    body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    ids = asset_cfg.joint_ids
    dev = asset.data.joint_pos[:, ids] - asset.data.default_joint_pos[:, ids]
    names = asset.joint_names if ids == slice(None) else [asset.joint_names[i] for i in ids]
    scale = torch.tensor([moving_thigh_scale if "thigh" in nm else 1.0 for nm in names], device=env.device)
    moving = torch.logical_or(cmd > 0.0, body_vel > velocity_threshold)
    return torch.where(moving, torch.linalg.norm(dev * scale, dim=1), stand_still_scale * torch.linalg.norm(dev, dim=1))
