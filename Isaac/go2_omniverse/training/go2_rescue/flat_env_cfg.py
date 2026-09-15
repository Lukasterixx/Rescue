# A flat-ground variant of the Rescue Go2 task, for checking the reward set cheaply.
#
# WHY IT EXISTS. The rough task is ~5x the wall clock of the flat one — 3.28 s per
# iteration against 0.64 s, measured, because the height scanner fires 187 rays per robot
# per step (766k rays at 4096 envs) and a generated terrain mesh is far more expensive to
# raycast than a plane. That makes the rough run a ~1h45 commitment, which is a long time
# to wait to discover a reward term is mis-weighted.
#
# So this trains the SAME rewards on flat ground in ~20 minutes. It exists to answer
# questions about the REWARD SET — is the stamping gone, has the stance widened, does the
# trot hold — not to produce the deployed policy. The answer transfers because none of the
# reward terms here are terrain-dependent: that is exactly why `foot_clearance_reward`,
# the one Spot term that IS terrain-dependent, is absent from the set (see rough_env_cfg).
#
# IT INHERITS THE ROUGH CONFIG RATHER THAN RESTATING IT. Everything that matters — the
# reward set, the speed scaling, the widened yaw command range, `rel_standing_envs` — is
# defined once in `rough_env_cfg.py` and changed once. A flat variant that copied those
# values would drift from the rough one the first time either was tuned, and then the cheap
# check would stop predicting the expensive run, which is the only reason to have it.

from isaaclab.utils import configclass

from .rough_env_cfg import UnitreeGo2RescueRoughEnvCfg


@configclass
class UnitreeGo2RescueFlatEnvCfg(UnitreeGo2RescueRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Flat ground, and no terrain curriculum to promote through.
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.curriculum.terrain_levels = None

        # THE HEIGHT SCANNER STAYS. This is the one thing that must NOT be tidied away,
        # and it is what `UnitreeGo2FlatEnvCfg` does that makes it unusable here: it sets
        # `observations.policy.height_scan = None`, dropping the observation from 235 to 48.
        # Over a plane the scan reads a constant the policy learns to ignore, which costs a
        # little sample efficiency and buys two things — the observation stays identical to
        # the rough task, so a checkpoint from either loads in `go2_omniverse`, and the two
        # runs stay comparable to each other.


@configclass
class UnitreeGo2RescueFlatEnvCfg_PLAY(UnitreeGo2RescueFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
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

        # Same reason as the rough PLAY config: this class inherits the TRAINING flat config,
        # not the rough PLAY one, so the command-range curriculum has to be stood down here
        # too or everything played back is capped at the curriculum's starting +-0.3 m/s.
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
        self.curriculum.lin_vel_cmd_levels = None
