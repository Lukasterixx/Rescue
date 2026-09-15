"""Rescue's Go2 locomotion task: P2Dingo's Spot-reward Go2, retrained for Rescue's stairs.

Discovered automatically. `isaaclab_tasks/__init__.py` calls `import_packages` over its own
tree, so a package dropped (or symlinked -- see ../install_task.sh) under
`manager_based/locomotion/velocity/config/` is imported at startup and registers itself.
No Isaac Lab file needs editing, which is what keeps this task surviving an IsaacLab pull.

WHAT THIS IS. A copy of P2Dingo's `go2_p2dingo` task (Isaac/go2_omniverse/training in that
repo) taken on 2026-09-12, with the pieces that produced the 2026 policy's in-place turns
and leg posture left exactly as they were, and the ground, the curriculum's promotion
rule and three stair-specific reward terms changed. `rough_env_cfg.py`'s header lists every
difference and the measurement behind it.
"""

import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Velocity-Rescue-Unitree-Go2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:UnitreeGo2RescueRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2RescuePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Velocity-Rescue-Unitree-Go2-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:UnitreeGo2RescueRoughEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2RescuePPORunnerCfg",
    },
)

# Flat variants exist for the posture/gait/turn probes, which take their height datum from
# the environment origin and would read terrain relief as gait on generated ground. They
# keep the height scanner, so a checkpoint from any of the four ids loads in the sim.
gym.register(
    id="Isaac-Velocity-Rescue-Unitree-Go2-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:UnitreeGo2RescueFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2RescueFlatPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Velocity-Rescue-Unitree-Go2-Flat-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:UnitreeGo2RescueFlatEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2RescueFlatPPORunnerCfg",
    },
)

# Short fine-tune experiments (experiments.py): Isaac-Velocity-Rescue-Exp-<Arm>-v0. Listed by name
# rather than imported, so registering them stays as cheap as registering the task above.
for _arm in ("Control", "Com", "Riser", "StairFwd", "Placement", "Thigh", "PlacementW30"):
    _cls = f"Exp{_arm}EnvCfg"
    gym.register(
        id=f"Isaac-Velocity-Rescue-Exp-{_arm}-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.experiments:{_cls}",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2RescueExpPPORunnerCfg",
        },
    )

# Flat ablations (experiments.py, bottom): Isaac-Velocity-Rescue-FlatAbl-<Arm>-v0, logged under go2_rescue_flat.
for _arm in ("FlatP2D", "FlatNoDR", "FlatNoArm", "FlatNoAct", "FlatClear08"):
    gym.register(
        id=f"Isaac-Velocity-Rescue-FlatAbl-{_arm}-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.experiments:{_arm}EnvCfg",
            "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2RescueFlatPPORunnerCfg",
        },
    )
