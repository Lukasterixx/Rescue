# RESCUE'S GO2 LOCOMOTION TASK. A copy of P2Dingo's `go2_p2dingo/rough_env_cfg.py`, taken
# 2026-09-12 after that task gained its path-length terrain curriculum and per-foot ground
# datum, with these differences and nothing else:
#
#   1. THE GROUND. `terrain_cfg.py` replaces the stock rough generator with one built around
#      Rescue's maze: 10 cm risers on 20 cm treads (the stock stairs are 30 cm), up and down.
#      No walls: avoiding them is the nav stack's job, not the gait's. Set in __post_init__
#      below, AFTER the parent runs, because the parent's __post_init__ indexes the stock
#      generator's sub-terrains by name.
#   2. NO EXTRA STAIR TERMS. Two were tried (a calf/thigh contact penalty and a riser-strike
#      penalty) and removed after measurement -- see the note where they used to live, above
#      `gait_pair_duty`. They charged the robot for the act of climbing and were too small to
#      matter anyway.
#   3. THE CLEARANCE TARGET is 10 cm, the riser height, rather than 8 cm.
#   4. THE ARM IS ON. The training robot is the Go2 with the D1 welded to it, built by the
#      sim's own `arm_weld.build_welded_robot_usd` at the same mount, mass and drive gains
#      the sim uses under `--arm_mount weld` (welded_robot.py). Actions, joint observations
#      and every joint-space penalty are scoped to the 12 leg joints, so the observation
#      stays 235 wide; three of Spot's penalties ignore that scoping and are replaced by
#      leg-scoped copies (mdp/rewards.py). Base-mass randomisation stays the stock (-1, +3)
#      kg, now on top of the real arm rather than standing in for it.
#   5. A 4000-ITERATION BUDGET, in agents/rsl_rl_ppo_cfg.py.
#
# WHAT IS DELIBERATELY IDENTICAL: the command generator (in-place turns, no heading control),
# the hip-abduction, posture, ride-height, gait, air-time, pair-duty and limit terms and
# every one of their weights -- those are what produced the 2026 policy's turns and stance,
# and the point of this task is to keep them while adding stairs. The evidence for each
# change is in ../README.md. P2Dingo's own reasoning follows, unedited except for names.
#
# P2Dingo's Go2 locomotion task: the stock Isaac Lab Go2, with Spot's reward set.
#
# WHY THIS EXISTS. The policy the sim ships (`model_7850_converted.pt`, trained 2024-04-06
# on `Isaac-Velocity-Rough-Unitree-Go2-v0`) walks with its feet under the robot's centre
# line and turns unstably. That is not a training-length problem, it is what the stock Go2
# reward set pays for:
#
#   * THE NOMINAL STANCE IS NOT DEFENDED. UNITREE_GO2_CFG's default pose abducts the hips by
#     0.1 rad (5.7 deg) and NOTHING in the stock reward holds them there — there is no
#     joint_deviation term of any kind. Collapsing the hips to zero is strictly cheaper
#     under dof_torques_l2 and action_rate_l2, so the reward actively buys the narrow
#     stance, and a narrow stance is what makes a turn unstable.
#   * THERE IS NO GAIT TERM. Nothing asks for diagonal pairs, so a turn has no reason to
#     stay a trot and degenerates into a shuffle.
#   * YAW TRACKING IS WEIGHTED HALF OF LINEAR (0.75 vs 1.5) AND USES A SQUARED KERNEL.
#     `track_ang_vel_z_exp` is exp(-err^2/std^2) with std^2 = 0.25, whose gradient VANISHES
#     as the error goes to zero — so past a certain accuracy there is no pressure left to
#     close the gap and the torque penalties win. Measured against this policy in the sim:
#     a commanded 0.929 rad/s is executed at ~0.515, i.e. ~55%, and it saturates near
#     0.55 rad/s however hard it is asked.
#
# Isaac Lab already ships the fix, for a different robot. Spot is the one quadruped it
# ships a good pretrained policy for, and Spot's reward set has all three of the missing
# pieces. Those terms are plain functions of state, so they port to the Go2 by changing
# body and joint names — which is all this file does.
#
# WHAT IT DELIBERATELY DOES NOT CHANGE: the observation space. `go2_omniverse`'s
# `custom_rl_env.py` is a term-for-term copy of `UnitreeGo2RoughEnvCfg`'s observations,
# height_scan included (235-dim), and a checkpoint is only loadable into an env whose
# observation vector matches the one it was trained on. So this class inherits the stock
# Go2 config, lets its `__post_init__` run in full, and then swaps ONLY the reward and
# command config. Touch `observations` here and the trained policy becomes unloadable in
# the sim, which is the one failure that would waste the whole training run.

import copy

from isaaclab.managers import CurriculumTermCfg, EventTermCfg, RewardTermCfg, SceneEntityCfg
from isaaclab.managers import TerminationTermCfg
from isaaclab.utils import configclass

import isaaclab.envs.mdp as base_mdp

import isaaclab_tasks.manager_based.locomotion.velocity.config.spot.mdp as spot_mdp
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import (
    UnitreeGo2RoughEnvCfg,
)

from . import mdp as rescue_mdp
from .terrain_cfg import RESCUE_TERRAINS_CFG
from .welded_robot import LEG_JOINTS, build_welded_go2_cfg, scope_to_legs

# The commanded speed at which the gait and air-time rewards reach full value. Below it
# they ramp down linearly, so a walk is not paid the same as a run and a standstill is
# paid nothing at all. 0.5 sits under everything Nav2 actually commands — DWB's floors are
# min_speed_xy 0.3 m/s and min_speed_theta 0.8 rad/s — so real mission motion is always at
# or near full scale and only the near-zero band is damped.
GAIT_FULL_SPEED = 0.5

# Go2 foot bodies are FL_foot / FR_foot / RL_foot / RR_foot. A trot syncs the DIAGONAL
# pairs, so these two tuples are what make the gait a trot rather than a pace or a bound —
# getting them wrong trains a perfectly good gait that is the wrong gait.
GO2_TROT_PAIRS = (("FL_foot", "RR_foot"), ("FR_foot", "RL_foot"))


@configclass
class Go2RescueRewardsCfg:
    """Spot's reward set, renamed onto the Go2.

    Weights are Spot's, unchanged, and that is deliberate: they are a proven, balanced set
    and this is a port, not a fresh tuning exercise. The values below that are NOT Spot's
    are marked, and each one is a size difference rather than a preference.
    """

    # -- task rewards -----------------------------------------------------------------
    #
    # NOTE THE KERNEL. Spot's velocity rewards are exp(-|err|/std) — ABSOLUTE error, not
    # squared. That matters more than the weight does: an absolute kernel keeps a constant
    # non-zero gradient all the way down to zero error, so the policy is still being paid
    # to close the last 10% of tracking error. The squared kernel the stock Go2 uses goes
    # flat there and hands the argument to the torque penalties. This is the single change
    # most likely to fix the ~55% yaw tracking.
    base_linear_velocity = RewardTermCfg(
        func=spot_mdp.base_linear_velocity_reward,
        weight=5.0,
        # `std` 0.3, NOT SPOT'S 1.0. A kernel should be narrower than the range it grades and
        # this one was wider: exp(-|err|/1.0) over a command range of +-1.0 m/s scores a
        # 0.2 m/s shortfall at 0.82 and a 0.5 m/s one at 0.61, so most of the term's output
        # is indifference. Two measurements say it is costing real accuracy -- forward
        # tracking of 79% at 1.0 m/s, and 0.08-0.13 m/s of DRIFT while turning on the spot,
        # which is linear velocity error by another name and was priced at a 10% discount.
        #
        # A KERNEL SHOULD BE NARROWER THAN THE RANGE IT GRADES, and Spot's 1.0 was wider:
        # over a +-1.0 m/s command range it scores a 0.2 m/s shortfall at 0.82, so most of
        # the term's output was indifference and drift while turning was priced at a 10%
        # discount. 0.3 then overshot the other way. Measured across the three:
        #
        #   std    drift turning @ 1 rad/s    forward tracking @ 0.5 / 1.0 m/s
        #   1.0             0.087 m/s                  104 % / 79 %
        #   0.3             0.046                       79 % / 74 %   <- here
        #   0.5             0.067                      107 % / 92 %
        #
        # 0.3 IS KEPT DESPITE 0.5 TRACKING BETTER, and the reason is not in this table. At
        # 0.5 the limp returns unless `air_time_variance` also goes to 80, and that pairing
        # (run 2026-09-12_08-52-13) costs 11.5 cm of swing height against 9.1 and a 1.2-1.4 cm
        # ride-height sag against 0.5-0.8 -- regressions in two faults that were specifically
        # reported and specifically fixed earlier. Forward tracking is not: nav2 closes a
        # speed shortfall through its position loop and simply arrives slower.
        #
        # A narrow absolute kernel goes flat once the error is large -- at 0.3 a 0.6 m/s
        # shortfall scores 0.13 and everything worse scores about the same -- so on commands
        # the policy cannot yet satisfy it stops trying and banks the regularisation terms
        # instead. That is the same indifference the wide kernel had, moved to the other end
        # of the range. At 0.5 a 0.2 m/s error scores 0.67 and a 0.6 m/s one 0.30, which
        # discriminates where it matters without going flat where the policy is still
        # learning.
        #
        params={"std": 0.3, "ramp_rate": 0.5, "ramp_at_vel": 1.0, "asset_cfg": SceneEntityCfg("robot")},
    )
    base_angular_velocity = RewardTermCfg(
        func=spot_mdp.base_angular_velocity_reward,
        weight=5.0,  # EQUAL to linear, where the stock Go2 had 0.75 against 1.5
        # `std` 2.0 IS ALSO WIDER THAN ITS OWN RANGE (+-1.5 rad/s) and is left alone anyway,
        # because the measurement no longer justifies touching it: yaw tracking is 93-95%
        # across the whole range once the command distribution was fixed (mdp/commands.py).
        # It was the obvious suspect for poor turning and it was the wrong one -- the policy
        # was never asked to hold a turn, not insufficiently rewarded for it. Tighten toward
        # 1.0 only if a later run comes back short on yaw.
        params={"std": 2.0, "asset_cfg": SceneEntityCfg("robot")},
    )
    # THE GAIT TERM, and the largest weight in the set. Contact-timing only, so it is
    # terrain independent and safe on the rough generator this task trains on.
    gait = RewardTermCfg(
        func=rescue_mdp.GaitRewardScaled,
        weight=10.0,
        params={
            "std": 0.1,
            "max_err": 0.2,
            "velocity_threshold": 0.5,
            "synced_feet_pair_names": GO2_TROT_PAIRS,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("contact_forces"),
            "full_speed": GAIT_FULL_SPEED,
        },
    )
    air_time = RewardTermCfg(
        func=rescue_mdp.air_time_reward_scaled,
        weight=5.0,
        params={
            "full_speed": GAIT_FULL_SPEED,
            # 0.25 s, not Spot's 0.3. This is the target air/contact duration, i.e. half a
            # gait period, and the Go2 is roughly half Spot's size and steps faster. First
            # thing to raise if the gait comes out mincing.
            "mode_time": 0.25,
            "velocity_threshold": 0.5,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )
    # THE SWING ARC. This term has been through two revisions and the reasoning behind both
    # is in `foot_clearance_reward_arc`'s docstring; the short version is that a clearance
    # reward paid against a CONSTANT height has its optimum at "snap up, hang, drop", which
    # is exactly what the policy trained to (58% of the swing above 80% of peak height, peak
    # vertical speed 2.6x the mean, the last 18% of the swing falling at ~1.9 m/s). A target
    # that follows `sin(pi * phase)` asks for the arc instead, and gating the term on being
    # AIRBORNE rather than on horizontal speed closes the hole that let a non-lifting foot
    # score full marks.
    #
    # `swing_time` is the seed used before a foot has completed a swing; after that the
    # phase denominator is the foot's own last air time, so the arc peaks at ITS mid-swing
    # rather than at a constant's. Keep the seed tracking `air_time.mode_time` above.
    #
    # WEIGHT 1.0 AND `std` 0.02 ARE THE SETTINGS THAT TRAINED THE INSTALLED POLICY, and two
    # attempts to strengthen the term from here both produced a WORSE swing -- the table and
    # the reason are in `foot_clearance_reward_arc`. Short version: every other term in the
    # set penalises vertical motion, so raising this one buys a flatter, lower arc rather
    # than a rounder one. Do not raise it without reading that note.
    foot_clearance = RewardTermCfg(
        func=rescue_mdp.foot_clearance_reward_arc,
        weight=1.0,
        params={
            "std": 0.02,
            # 0.10, NOT P2DINGO'S 0.08 -- THE RISER HEIGHT. Under this set the arc's optimum
            # lands near the target's mean (target * 2/pi, see the function): ~5 cm at 0.08,
            # ~6.4 cm at 0.10. Neither clears a 10 cm riser by itself; the terrain and the
            # stumble term teach that. This only points the reward's preference the right
            # way instead of against it. Measure with measure_swing.py before raising it.
            "target_height": 0.10,
            "swing_time": 0.25,
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("height_scanner"),
            "contact_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )

    # THE SPIDER TERM, AND THE ONLY OWNER OF THE ABDUCTION JOINTS. `joint_pos` below takes
    # ONE norm across the joints it is given, so a hip splayed 0.2 rad out would be averaged
    # in with the others and barely register; this names the four abduction joints —
    # `_hip_joint` is the roll DOF on a Go2 — and prices them on their own. The literature
    # calls this a hip joint position or hip symmetry reward and introduces it for exactly
    # this symptom: outward thigh abduction during fast locomotion.
    #
    # TWO CHANGES FROM THE FIRST VERSION, both measured off the policy it trained
    # (`measure_posture.py`, 32 envs, 250 steps per command, flat):
    #
    #   command      hip abduction   swing peak-to-peak   worst   foot out   body height
    #   stand              4.60 deg             0.03 deg  9.0 deg   16.6 cm       32.9 cm
    #   walk 0.5 m/s       5.66 deg            11.64 deg 27.6 deg   17.0 cm       35.4 cm
    #   walk 1.0 m/s       6.19 deg            18.15 deg 35.6 deg   17.1 cm       34.9 cm
    #   turn 1.0 rad/s     5.80 deg            12.11 deg 19.7 deg   17.1 cm       35.8 cm
    #
    # FIRST, THE TARGET IS NOW ZERO RATHER THAN THE DEFAULT POSE. `joint_deviation_l1`
    # measures against `default_joint_pos`, which abducts every Go2 hip by 0.1 rad (5.73
    # deg) — so the old term was not holding the legs vertical, it was DEFENDING a 5.7 deg
    # splay. The measurement is unambiguous: the hips rest at 4.60 deg, inside the 5.73 the
    # pose asks for, so the lean visible at a standstill is the target being met, not missed.
    # At zero abduction the leg lies in its own sagittal plane and the foot sits under the
    # thigh mount: 14.2 cm out, 28.4 cm of stance, still far wider than the 19.1 cm between
    # the hips and nothing like the centreline stance of the 2024 policy.
    #
    # SECOND, -0.75 -> -2.0, which is what acts on the SWING. Nothing in the set paid for
    # abduction, so 18 deg of it per stride was simply unpriced freedom; an L1 penalty costs
    # a 35 deg excursion five times what it costs to stay inside 7, and that asymmetry is
    # the whole mechanism.
    #
    # WHAT THE TWO CHANGES BOUGHT, re-measured the same way after a 2000-iteration flat run:
    #
    #   command        worst abduction     resting splay    foot side-travel   yaw tracking
    #   stand           9.0 ->  12.6 deg   4.67 -> 1.94 deg
    #   walk 0.5 m/s   32.0 ->   9.9 deg                    5.48 -> 2.92 cm
    #   walk 1.0 m/s   36.4 ->  10.7 deg                    8.04 -> 6.03 cm    95.1 -> 95.4%
    #   turn 1.0 rad/s 21.5 ->  11.9 deg                    6.36 -> 6.75 cm    85.8 -> 89.7%
    #
    # The oscillation now sits CENTRED ON VERTICAL instead of biased outward — at a turn its
    # mean is 0.95 deg INSIDE vertical — which is why the worst excursion falls by 3.4x while
    # the peak-to-peak falls by less. And the narrower stance cost nothing: yaw tracking
    # improved, which was the risk worth taking seriously, since turn stability was the
    # reason this reward set exists at all.
    hip_deviation = RewardTermCfg(
        func=rescue_mdp.hip_abduction_penalty,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*_hip_joint"), "target": 0.0},
    )

    # RIDE HEIGHT, MEASURED AGAINST THE TERRAIN. Absent from Spot's set entirely, and the
    # table above says what that costs: the body stands at 32.94 cm and walks at 34.9-35.8,
    # so it visibly drops every time the robot stops. Targeting the DEFAULT POSE's own height
    # (32.94 cm measured, 0.33 set) is what makes this term agree with `joint_pos` rather
    # than fight it — the standstill is already there, and it is the walk that gets pulled
    # back into line.
    #
    # MEASURED AFTER: the stand-to-walk gap closes from 2.2-2.6 cm to 1.0-1.6 cm and the bob
    # amplitude with it (sd 1.72 -> 1.05 cm at 0.5 m/s). Better, not gone — the body still
    # rides its trot slightly high, and the residual is what a heavier weight would buy. It
    # is deliberately not chased further here: at std 0.02 the term already has half its
    # gradient left on the table at that offset and is not taking it, which says something
    # else in the set is paying for the extra centimetre (`foot_clearance` and `air_time`
    # both get easier the higher the hips sit), so the honest next move is to look there
    # rather than to keep raising this.
    base_height = RewardTermCfg(
        func=rescue_mdp.base_height_reward_terrain,
        weight=1.5,
        params={
            "target_height": 0.33,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("height_scanner"),
        },
    )

    # -- regularisation penalties -----------------------------------------------------
    #
    # THE POSTURE TERM. Penalises the thigh and calf joints' deviation from
    # UNITREE_GO2_CFG's default pose, which is what stops the legs folding up or reaching
    # out into something that is not a Go2 stance, and what sets the ride height the
    # `base_height` term above is targeted at. `stand_still_scale` multiplies the penalty
    # when no motion is commanded, so standing still is held tightly while walking is left
    # room to move. If the gait comes out stiff or short-strided, this weight is the cause
    # and the first one to back off.
    #
    # It used to run over `.*` — all twelve joints — and pulling the hips toward a default
    # that is 0.1 rad abducted is exactly the splay `hip_deviation` now exists to remove.
    #
    # AND IT USES A LOCAL COPY OF THE FUNCTION, because Spot's own ignores `joint_ids`
    # entirely (see `joint_position_penalty_scoped`). Naming joints against the upstream
    # term does nothing, silently — the first version of this narrowing was a no-op and the
    # hips were still being fought over.
    joint_pos = RewardTermCfg(
        func=rescue_mdp.joint_position_penalty_scoped,
        weight=-0.7,
        params={
            # THIGH AND CALF ONLY — the hips are `hip_deviation`'s, and this term must not
            # be given a say in them. It measures against the default pose, which abducts by
            # 0.1 rad, so including the hips here would pull them straight back out to the
            # 5.7 deg splay the term above exists to remove. Two terms disagreeing about the
            # same joint is how a reward set ends up at an equilibrium neither wanted.
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_thigh_joint", ".*_calf_joint"]),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
        },
    )
    # THE HARD STOP. Measured on the 20-21-44 policy at 1.0 m/s: the calf reaches 0.0% of
    # its joint range from the limit — it is being driven into its mechanical stop mid-trot.
    # That is a discontinuity in the leg's motion no other term is watching, it is one of the
    # things that makes the swing look un-cyclic, and on real hardware it is an impact.
    #
    # Nothing in Spot's set covers it: Spot's joints are not driven to their stops, so the
    # port inherited no limit term. The stock Go2 set does carry one — as
    # `dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=0.0)` — DISABLED at weight
    # zero, which is how it was possible to drop the whole rewards block without noticing a
    # term had gone missing.
    #
    # `joint_pos_limits` is one of the few upstream penalties that DOES honour `joint_ids`,
    # and it only fires past the soft limits (0.9 of the range, per the Go2 asset's
    # `soft_joint_pos_limit_factor`), so it costs nothing until the leg is already somewhere
    # it should not be. All twelve joints: the calf is the one measured at the stop, but a
    # term that exists to catch mechanical abuse should not be scoped to the joint that
    # happened to show it first.
    joint_limits = RewardTermCfg(
        func=base_mdp.joint_pos_limits,
        weight=-5.0,
        # LEGS ONLY, where P2Dingo has `.*`: the arm's joints are on this robot too, and a
        # folded arm sits nowhere near its limits, so this is for honesty rather than effect.
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    foot_slip = RewardTermCfg(
        func=spot_mdp.foot_slip_penalty,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "threshold": 1.0,
        },
    )
    # -- Rescue: stairs ----------------------------------------------------------------
    #
    # TWO TERMS WERE HERE AND ARE GONE, and the reason is measured rather than stylistic.
    # `undesired_contacts` on calves and thighs (-1.0) and `foot_stumble` (-0.5) were added on
    # the reasoning that a shin on a step edge and a toe against a riser are how a climb
    # stalls. Both were plausible and both were wrong, twice over.
    #
    # FIRST, THEY PENALISE THE ACT OF CLIMBING. On a 20 cm tread the shin has nowhere to be
    # except near the edge, and stepping onto a riser BEGINS with the toe touching its face --
    # precisely what the stumble detector is built to catch. Isaac Lab's own Go2 rough config
    # disables `undesired_contacts` for this robot, and that was overridden here on a hunch.
    #
    # SECOND, AND WHY THEY ARE MERELY GONE RATHER THAN RETUNED: they were never big enough to
    # matter either way. `training/measure_climbcost.py`, attributing reward to what the robot
    # actually did over 12 s, found a climbing robot earns 45.6 against 74.8 for one on level
    # ground -- and of that 29-unit gap these two terms account for 0.15, half of one percent.
    # The gap is the POSITIVE terms collapsing: gait -27.2, velocity tracking -9.6. Climbing is
    # unprofitable under a reward that pays for horizontal speed, and no penalty tweak changes
    # that. What changes it is terrain the policy can learn on by degrees -- see terrain_cfg.py.

    # THE LIMP TERM, AND ITS WEIGHT IS 30x SPOT'S FOR A UNITS REASON, not a preference.
    #
    # `air_time_variance_penalty` is the variance ACROSS FEET of the last air and contact
    # durations, which is precisely the right measurement for a gait that loads one diagonal
    # pair harder than the other. Measured on 2026-09-11_23-48-25:
    #
    #   command        FL stance   FR stance   pair duty FL+RR / FR+RL
    #   walk 0.5 m/s      121 ms      326 ms         44 % / 81 %      (2.7x)
    #   walk 1.0 m/s      122 ms      250 ms         49 % / 76 %      (2.1x)
    #   turn 1.0 rad/s    107 ms      396 ms         22 % / 81 %      (3.7x)
    #
    # Total ground contact sums to 250% rather than a trot's 200%: it bears weight on FR+RL
    # and uses FL+RR as a quick light support. The trot PHASE stays textbook throughout,
    # which is why pooled duty and phase metrics miss it completely.
    #
    # The term saw all of this and charged -0.011 per step. A spread of +-0.10 s gives a
    # variance of 0.0104 per quantity, about 0.02 for air and contact together, and at
    # Spot's weight of 1.0 that is three orders of magnitude below the gait term it has to
    # argue with. Variance of seconds is simply a small number; the weight has to carry the
    # unit conversion.
    #
    # 30 IS ENOUGH ONLY IN COMPANY. It takes the pair ratio from 1.84x to 1.15x at a walk and
    # 1.06x at 1 m/s -- but only while `base_linear_velocity.std` stays at 0.3. Widening that
    # to 0.5 brought the limp back to 1.32x and brought it back MIRRORED: FL+RR became the
    # long pair (336 ms vs 201) where before they were the short one. A fault that flips sign
    # between runs is a symmetry the policy breaks arbitrarily, and at 30 the term charged
    # 0.235 and 0.257 in those two runs -- nearly the same number for visibly different
    # gaits, so it was not the thing holding it.
    #
    # 80 DOES HOLD IT AT std 0.5 (ratio 1.07x, run 2026-09-12_08-52-13) and is not used,
    # because it pays for the symmetry with 11.5 cm of swing height and a 1.4 cm ride-height
    # sag. The three-way comparison is in training/README.md.
    #
    # Because variance is QUADRATIC in the spread this term fades on its own as the gait
    # evens out -- a residual +-0.02 s costs 0.024 -- which is what allows a weight well
    # above Spot's without it dominating a gait that is already symmetric. It also cannot be
    # satisfied by pronking, the degenerate zero-variance gait, because `gait`'s async
    # component prices that far more heavily.
    #
    # If a run still comes back limping, the next lever is `gait.max_err` (0.2): Spot's gait
    # reward clips its squared timing error at `max_err**2`, and the differences measured
    # above are 0.12-0.29 s, so that term is saturated and has no gradient left to give.
    # THE DIAGONAL PAIRS MUST CARRY THE ROBOT FOR EQUAL FRACTIONS OF THE STRIDE. `gait`
    # above grades only the PHASE relationships and a limp satisfies it perfectly -- see
    # mdp/rewards.py, which also records how the first version of this term wrecked a full
    # 90-minute run by measuring the imbalance in SECONDS instead of as a fraction.
    #
    # -5.0 against the fraction: v1's turn imbalance was 0.304 (66.4% against 36.0%) and its
    # 1.0 m/s walk 0.067, so this is ~1.5 of pressure where the limp is and ~0.3 where the
    # gait is already balanced -- the sizing the seconds version was meant to have.
    gait_pair_duty = RewardTermCfg(
        func=rescue_mdp.gait_pair_duty_penalty,
        weight=-5.0,
        params={
            "pair_a_cfg": SceneEntityCfg("contact_forces", body_names=list(GO2_TROT_PAIRS[0])),
            "pair_b_cfg": SceneEntityCfg("contact_forces", body_names=list(GO2_TROT_PAIRS[1])),
            "full_speed": GAIT_FULL_SPEED,
        },
    )

    air_time_variance = RewardTermCfg(
        func=spot_mdp.air_time_variance_penalty,
        weight=-30.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
    )
    base_motion = RewardTermCfg(
        func=spot_mdp.base_motion_penalty,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
    # The stock Go2 ran flat_orientation_l2 at weight 0.0, i.e. off. A body that is allowed
    # to roll is a body whose feet are not where the policy thinks they are.
    base_orientation = RewardTermCfg(
        func=spot_mdp.base_orientation_penalty,
        weight=-3.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
    action_smoothness = RewardTermCfg(func=spot_mdp.action_smoothness_penalty, weight=-1.0)
    joint_torques = RewardTermCfg(
        func=rescue_mdp.joint_torques_penalty_scoped,
        weight=-5.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    # THE THREE NORM PENALTIES ARE LEG-SCOPED COPIES OF SPOT'S. P2Dingo spells these `.*`
    # and says why: Spot's `joint_acceleration_penalty` and `joint_velocity_penalty` (and
    # `joint_torques_penalty` above) norm over the WHOLE joint array and never read the ids,
    # so on the bare Go2 the scope is decorative. On the welded robot it is not -- `.*` is
    # twenty joints, six of which are servos holding an arm -- so the same maths is
    # re-implemented over the twelve legs (mdp/rewards.py). The calf stays in, deliberately:
    # it is the jerkiest joint on the robot and these weights were sized with it included.
    joint_acc = RewardTermCfg(
        func=rescue_mdp.joint_acceleration_penalty_scoped,
        weight=-1.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    joint_vel = RewardTermCfg(
        func=rescue_mdp.joint_velocity_penalty_scoped,
        weight=-1.0e-2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )


@configclass
class UnitreeGo2RescueRoughEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        # THE PARENT RUNS FIRST, IN FULL, AND THEN THE REWARDS ARE REPLACED. Not the other
        # way round and not by overriding the `rewards` field: `UnitreeGo2RoughEnvCfg`'s
        # own __post_init__ reaches into `self.rewards.feet_air_time`,
        # `self.rewards.undesired_contacts` and three more by name, so a config that has
        # already swapped them out raises AttributeError before anything else happens.
        # Running the parent untouched is also what guarantees the scene, the actions
        # (joint_pos.scale 0.25), the events, the terminations and — the one that matters —
        # the OBSERVATIONS are bit-for-bit the stock Go2's.
        super().__post_init__()

        self.rewards = Go2RescueRewardsCfg()

        # THE COMMAND TERM IS REPLACED, not just re-ranged. `Go2P2DingoRewardsCfg` is only
        # half the story: the stock command generator never asks for a sustained turn, so no
        # reward weight could have taught one. See `mdp/commands.py` for the measurements --
        # the short version is that `heading_command=True` overwrites the sampled yaw every
        # step with a P-controller on heading error, which decays to zero as the robot turns.
        #
        # `heading_command=False` is also the honest model of deployment: DWB publishes an
        # angular velocity on /cmd_vel and there is no heading controller between it and the
        # policy. Training against one means training against a command distribution the
        # robot will never see.
        turning = rescue_mdp.TurnInPlaceVelocityCommandCfg(
            asset_name="robot",
            resampling_time_range=(10.0, 10.0),
            heading_command=False,
            debug_vis=True,
            # THE LINEAR RANGE STARTS NARROW AND IS EARNED OUTWARD. `ranges` is what is
            # sampled now; `limit_ranges` is where the curriculum is allowed to take it. See
            # `mdp/curriculums.py` for why: sampling the full +-1.0 from iteration 0 spends
            # the fast end of the range on a policy that cannot yet walk, and the gait that
            # survives is the one optimised for the middle -- which is what 83.6 % tracking
            # at 1 m/s against 91.5 % at 0.5 m/s looks like.
            ranges=rescue_mdp.TurnInPlaceVelocityCommandCfg.Ranges(
                lin_vel_x=(-0.3, 0.3),
                lin_vel_y=(-0.3, 0.3),
                # YAW IS NOT CURRICULUMISED, so this is its final range from iteration 0.
                # It is the one axis that already tracks at 94-104 % across the board, and a
                # curriculum can only cost samples on an axis that has nothing left to fix.
                #
                # WIDER THAN THE STOCK (-1.0, 1.0), because Nav2 asks for more: DWB commands
                # up to `max_vel_theta`, which nav2_params.yaml sets to 1.3 rad/s, and a
                # policy trained only to +-1.0 extrapolates exactly when the robot is turning
                # hardest. With `heading_command` off this is finally a SAMPLED range rather
                # than a clip on the heading controller's output, which is what it was before.
                ang_vel_z=(-1.5, 1.5),
            ),
            limit_ranges=rescue_mdp.TurnInPlaceVelocityCommandCfg.Ranges(
                lin_vel_x=(-1.0, 1.0),
                lin_vel_y=(-1.0, 1.0),
                ang_vel_z=(-1.5, 1.5),
            ),
        )
        self.commands.base_velocity = turning

        # STANDING STILL HAS TO BE TRAINED, and at the stock 0.02 it essentially was not:
        # two environments in a hundred were ever commanded to hold position, so the policy
        # had almost no experience of the one case where the right answer is to do nothing.
        # Together with the speed-scaled gait reward above, this is what stops the stamping
        # on the spot. 0.15 is a fifth of the way to Isaac Lab's own standing-heavy configs
        # and still leaves 85% of the batch learning to move.
        self.commands.base_velocity.rel_standing_envs = 0.15

        # ONE ENVIRONMENT IN SEVEN TURNS ON THE SPOT -- yaw commanded, both linear terms
        # forced to zero. Without this, a pure spin needs both linear samples to land near
        # zero at once, which independent uniforms deliver about 1% of the time, and the
        # policy has to extrapolate into the one manoeuvre every label read ends with.
        self.commands.base_velocity.rel_turning_envs = 0.15

        # THE TERRAIN CURRICULUM IS RE-RULED. `terrain_levels_vel` judges a robot by its
        # straight-line displacement from spawn, which measures locomotion only while
        # heading control forces everything to walk straight -- and `heading_command` is off
        # here on purpose. A held yaw command traces a circle, a circle has no displacement,
        # and the stock rule then demotes faster than it promotes even with a perfect policy
        # (28.5% up against 47.2% down, measured). That is why the v1 rough run finished at
        # terrain level 1.45 of 10, having seen 3.3 degree slopes. See mdp/curriculums.py.
        # NOT `terrain_levels_path`, which is P2Dingo's and is a ONE-WAY RATCHET on stairs:
        # it promotes on distance walked, and a robot circling the spawn platform banks that
        # without touching a step. Measured: 85.7 % promoted, 0.0 % demoted, and the run it
        # produced climbed nothing. See mdp/curriculums.py.
        self.curriculum.terrain_levels = CurriculumTermCfg(func=rescue_mdp.terrain_levels_traverse)

        # THE RANGE ABOVE IS WALKED OUTWARD BY THIS TERM, not left where it starts. Without
        # it the config trains at +-0.3 m/s and nothing says so -- so if this is ever removed,
        # put `ranges` back to `limit_ranges` in the same edit.
        self.curriculum.lin_vel_cmd_levels = CurriculumTermCfg(
            func=rescue_mdp.lin_vel_cmd_levels,
            params={
                # MEAN TRACKING ERROR AS A FRACTION OF MEAN COMMANDED SPEED. A policy that
                # ignores its command scores 1.0 at any range; the 2026-09-12_11-46-13 rough
                # policy scores 0.27 at full range, and 0.91 at iteration 400 when it still
                # could not walk. 0.55 sits between them with margin on both sides, and --
                # unlike a threshold on the reward or on the raw error -- means the same thing
                # at +-0.3 as it does at +-1.0. See mdp/curriculums.py.
                "error_ratio_threshold": 0.55,
                "step": 0.1,
                # 15000 env steps is ~625 iterations at num_steps_per_env=24, so the range is
                # full by ~920 iterations even if the gate never once passes, leaving over half
                # the run at the full range. A curriculum that stalls should cost some training
                # time, not the whole point of the training.
                "force_full_after_steps": 15000,
            },
        )

        # THE CENTRE OF MASS IS RANDOMISED AGAIN. The stock Go2 rough config turns this off
        # (`self.events.base_com = None` in UnitreeGo2RoughEnvCfg), so every robot in the
        # batch is trained on an exactly nominal mass distribution -- and the real Go2 is not
        # one: it carries the Livox, the mount and the camera head forward of the nominal
        # point. Biasing the offset toward +x (the head) rather than randomising symmetrically
        # is the part that matters; the range is deliberately small because this is meant to
        # widen the policy's basin, not to teach it a different robot.
        self.events.base_com = EventTermCfg(
            func=base_mdp.randomize_rigid_body_com,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names="base"),
                # SYMMETRIC ABOUT THE NOMINAL ROBOT SINCE 2026-09-14, was x (0.0, 0.05). The sim does not
                # shift the CoM, so a forward-only range left the sim's robot at the very edge of
                # the training distribution, and forward speed followed the offset almost linearly:
                # 80 % tracking at 0 cm, 103 % at +2.5, 116 % at +5 (measure_bench.py). Two fine-tunes
                # with this range tracked at 107-108 %. Bias it forward again for the real robot.
                "com_range": {"x": (-0.03, 0.03), "y": (-0.02, 0.02), "z": (-0.02, 0.02)},
            },
        )

        # ------------------------------------------------------------------ Rescue --
        #
        # THE GROUND IS RESCUE'S. Everything the parent chain did to the stock generator
        # (the Go2 scaling of `boxes` and `random_rough`, `curriculum = True`) happened to
        # the object it had at the time; this one carries those values itself
        # (terrain_cfg.py) and has to be told about the curriculum again, or every robot
        # trains on a difficulty drawn at random and `Curriculum/terrain_levels` means
        # nothing.
        # A copy, so the PLAY config's edits to num_rows/num_cols/curriculum never reach the
        # module-level object that the training config reads.
        self.scene.terrain.terrain_generator = copy.deepcopy(RESCUE_TERRAINS_CFG)
        self.scene.terrain.terrain_generator.curriculum = True

        # THE ARM IS WELDED ON, AS THE SIM RUNS IT. The stock Go2 config the parent installed
        # is replaced by the same Go2 pointed at a freshly composed Go2+D1 USD -- the sim's
        # own weld, same mount, same 3.152 kg mass model, same servo gains -- and then every
        # joint-space action and observation is narrowed back to the twelve legs so the
        # policy sees the robot the checkpoint expects. welded_robot.py has the reasoning
        # and the guard. The stock base-mass randomisation (-1, +3 kg) is left as it is: it
        # now sits on top of the real arm, as it should, rather than standing in for it.
        self.scene.robot = build_welded_go2_cfg(self.scene.robot)
        scope_to_legs(self)

        # ----------------------------------------------------- domain randomisation --
        #
        # FOUR SETTINGS TAKEN FROM MaiRo's `unitree_rl_lab` (mairo-rl-lab-rinam), a
        # Unitree-derived stack with demonstrated transfer onto this robot. Its reward set is
        # close to stock and is NOT what makes it transfer; these four and the motor model in
        # welded_robot.py are. None of them touches the observation, the action or the
        # network, so the result stays a drop-in for the sim.
        #
        # They cost training time, and that is the point: each one removes an assumption the
        # policy would otherwise be free to rely on. Expect the terrain curriculum to climb
        # more slowly than it would without them.

        # 1. THE GROUND IS NOT ONE SURFACE. The stock config pins the robot's feet at 0.8
        # static / 0.6 dynamic friction -- a single number, not a range, so the policy may
        # learn a gait that is only stable at that grip. 0.3 is a wet floor, 1.2 is rubber on
        # dry concrete, and the maze is neither everywhere. Friction combines multiplicatively
        # with the terrain's own 1.0, so these are the effective values. `make_consistent`
        # clamps dynamic <= static, which MaiRo leaves off and which is physically right: a
        # surface that grips harder once sliding does not exist.
        self.events.physics_material.params.update({
            "static_friction_range": (0.3, 1.2),
            "dynamic_friction_range": (0.3, 1.2),
            "restitution_range": (0.0, 0.15),
            "make_consistent": True,
        })

        # 2. THE ROBOT GETS SHOVED. `UnitreeGo2RoughEnvCfg` sets `push_robot = None`, so
        # nothing in the stock Go2 task ever disturbs the body -- and a policy that has never
        # been pushed has never had to recover. Re-created here rather than re-enabled,
        # because the parent destroyed the term. 1 m/s is a real shove for a 15 kg robot; on
        # stairs it is also the closest thing in training to a foot slipping off a tread.
        self.events.push_robot = EventTermCfg(
            func=base_mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=(5.0, 10.0),
            params={"velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0)}},
        )

        # 3. EPISODES DO NOT START FROM REST. The stock reset puts every joint at exactly the
        # default pose with exactly zero velocity, so the first steps of every episode are
        # drawn from a single point the real robot is never in -- it is always mid-motion,
        # settling, or being stood up. Position stays at the default (the parent sets
        # (1.0, 1.0)); only the velocities are randomised.
        self.events.reset_robot_joints.params["velocity_range"] = (-1.0, 1.0)

        # 4. A ROBOT ON ITS BACK IS NOT A TRAINING SAMPLE. `base_contact` only ends the
        # episode when the BASE is touched, so a robot that has rolled onto its side without
        # the base hitting anything keeps running -- collecting up to 20 s of gradient from a
        # state no recovery is possible from and the deployed robot will never be in. 0.8 rad
        # (46 deg) is past any real gait and well short of the pitch of the steepest stair.
        # It matters more here than in MaiRo's flat world, and more again with the arm's mass
        # raising the centre of gravity.
        self.terminations.bad_orientation = TerminationTermCfg(
            func=base_mdp.bad_orientation,
            # 1.0 rad (57 deg), not MaiRo's 0.8 (46 deg). Their robot trains on flat
            # ground where 46 deg is unambiguously a fall. A 13 cm riser on a 20 cm tread is
            # a 33 deg slope before any dynamic pitch, so 0.8 sat close enough to a climb to
            # risk cutting short the very episodes that were learning one. Untested as a
            # cause -- unlike the two terms above -- but cheap insurance.
            params={"limit_angle": 1.0},
        )


@configclass
class UnitreeGo2RescueRoughEnvCfg_PLAY(UnitreeGo2RescueRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # A small scene to watch a trained policy in, with the training-time randomisation
        # off so what is on screen is the policy rather than the noise.
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.scene.terrain.max_init_terrain_level = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None

        # THE SURFACE IS PINNED FOR PLAYBACK. Training randomises foot friction over
        # 0.3-1.2 so the policy cannot rely on one grip; a PLAY run wants to measure the
        # POLICY, and 32 robots each on a different surface is 32 different experiments.
        # Back to the stock fixed values, which is also what the 2026 policy's probe numbers
        # in ../../agent_cfg.py were measured at, so the two are comparable.
        self.events.physics_material.params.update({
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
        })

        # PLAY GETS THE FULL RANGE IMMEDIATELY. A trained policy is being watched, not
        # trained, so the curriculum's starting range would silently cap everything on screen
        # (and every probe that leans on the command generator) at +-0.3 m/s.
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
        self.curriculum.lin_vel_cmd_levels = None
