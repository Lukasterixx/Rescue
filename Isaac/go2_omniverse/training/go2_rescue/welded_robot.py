"""The training robot: the Go2 with the D1 welded on, exactly as the sim runs it.

WHY TRAIN WITH THE ARM ON. The sim runs `--arm_mount weld` (Isaac/go2_omniverse/arm_weld.py):
the D1 is folded into the Go2's articulation with a fixed joint, so its ~3.15 kg, its
inertia and its centre of mass above the back all reach the gait. A policy trained on the
bare Go2 meets that robot for the first time on deployment. The alternative -- randomising
base mass to cover the arm -- gets the kilograms right and the geometry wrong: an extra
3 kg AT the base is not 3 kg cantilevered 8 cm above it. So the training robot is built by
the same function the sim uses, from the same URDF, at the same mount, with the same mass
model, and the policy trains on what it will drive.

WHAT THE POLICY SEES IS UNCHANGED. The articulation has 20 joints (12 legs, 6 arm, 2 jaws)
and the checkpoint expects 12, so every joint-space term is scoped to LEG_JOINTS by
`scope_to_legs()` below, exactly as `omniverse_sim._scope_env_cfg_to_legs()` does at
playback. Scoping is safe because `SceneEntityCfg` resolves joint names through
`find_joints(..., preserve_order=False)`, which returns indices in ascending articulation
order: subsetting cannot reorder the legs relative to each other, so the 12 values arrive
in the order the checkpoint expects even though PhysX interleaves the arm's joints among
them. The observation stays 235 wide and the action 12, which is what keeps the result a
drop-in for `agent_cfg.py`.

THE ARM IS HELD FOLDED. Its joints are driven to zero by their own PD actuators (the sim's
weld gains) and are not in any action term, so a reset puts them at zero and they stay
there. That is the `return_to_zero` pose the sim starts in and the pose the arm rides in
while the dog walks. It is not randomised: holding a randomised pose would need the arm's
position targets rewritten every reset, and the deployment case is the folded one.

THE WELD IS THE GUARD. `ImplicitActuatorCfg(joint_names_expr=["Joint[1-6]"])` raises at
scene creation if no joint matches, so a weld that silently produced a second articulation
-- the failure `omniverse_sim._report_articulation` exists to catch -- cannot start a run.

Ported from Rescue's `omniverse_sim.setup_welded_arm()` and D1Training's
`flat_env_cfg.make_robot_cfg()`; the gains and their measurements are documented in both.
"""

from __future__ import annotations

import copy
import os
import sys
from pathlib import Path

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.managers import SceneEntityCfg

# The 12 joints the walking policy owns. Same regexes as custom_rl_env.LEG_JOINTS.
LEG_JOINTS = [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]

D1_ARM_JOINTS = [f"Joint{i}" for i in range(1, 7)]
D1_GRIPPER_JOINTS = ["Joint7_1", "Joint7_2"]

# Unitree's published D1-550 mass. The sim's `--arm_mass` default; the two must agree or
# the policy trains on a different payload from the one it drives.
D1_550_MASS_KG = 3.152

# Arm base offset from the Go2's base link, in metres. Rescue's ARM_MOUNT_Z.
ARM_MOUNT_Z = 0.08

# The sim's WELD gains (omniverse_sim.py, `--arm_mount weld` branch). These say how hard
# the servo chases its target; PhysX still clamps torque at the joint's published effort
# limit, so raising them buys tracking, never strength. 800 (the teleport value) behaves as
# a soft spring and droops several degrees per joint under gravity; 4000 models a servo
# that holds its angle, which is what a real D1 does. Measured in D1Training.
ARM_BASE_STIFFNESS = 4000.0
ARM_BASE_DAMPING = 400.0
GRIPPER_BASE_STIFFNESS = 4000.0
GRIPPER_BASE_DAMPING = 400.0

# Written beside the sim's own `go2_d1.usd` under a different name, so a training run and
# a sim started at the same time never overwrite each other's file mid-read. (Both write
# the intermediate `d1.usd` from the same URDF; identical bytes, so a clash there is
# harmless.)
TRAIN_USD_NAME = "go2_d1_train.usd"


def go2_omniverse_root() -> Path:
    """Isaac/go2_omniverse in the Rescue checkout, resolved through the install symlink."""
    return Path(__file__).resolve().parents[2]


def build_welded_go2_cfg(base_cfg: ArticulationCfg, arm_mass_kg: float = D1_550_MASS_KG,
                         measured_actuator: bool = True) -> ArticulationCfg:
    """`base_cfg` (the stock Go2) pointed at a freshly welded Go2+D1 USD, with arm drives.

    Everything else about the Go2 -- leg actuator model, rigid and articulation props,
    `activate_contact_sensors` (which the contact sensor needs on the arm's links too) --
    is left exactly as the stock config has it, so any change in the gait is attributable
    to the arm and not to a re-tuned quadruped.

    Must run after the Isaac Sim app is up: the weld composes USD stages and imports the
    URDF through a Kit extension. Every Isaac Lab entry point (train.py, play.py,
    list_envs.py) launches the app before importing tasks, so a config's __post_init__ is
    late enough.
    """
    root = go2_omniverse_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from arm_weld import build_welded_robot_usd  # noqa: E402  (needs pxr; app must be up)
    from go2_actuators import apply_go2_actuator  # noqa: E402

    weld = build_welded_robot_usd(
        go2_usd_path=base_cfg.spawn.usd_path,
        d1_urdf_path=os.path.join(root, "d1_arm", "d1.urdf"),
        out_usd_path=os.path.join(root, "d1_arm", "generated", TRAIN_USD_NAME),
        mount_pos=(0.0, 0.0, ARM_MOUNT_Z),
        arm_mass_kg=arm_mass_kg,
    )

    # `.copy()` is `dataclasses.replace`, which rebuilds the top-level object and SHARES
    # every nested one -- so `cfg.init_state` and `cfg.spawn` are still the module-level
    # `UNITREE_GO2_CFG`'s. Editing them in place would leak the arm's joints into every
    # other Go2 task in the process (`list_envs.py` imports them all). Deep-copy first.
    cfg = base_cfg.copy()
    cfg.init_state = copy.deepcopy(cfg.init_state)
    cfg.spawn = copy.deepcopy(cfg.spawn)
    cfg.spawn.usd_path = weld.usd_path

    # The arm starts folded at its zero pose; the legs keep the stock default pose.
    cfg.init_state.joint_pos = dict(cfg.init_state.joint_pos)
    cfg.init_state.joint_pos.update({"Joint[1-6]": 0.0, "Joint7_.*": 0.0})

    # THE LEGS GET THE MEASURED MOTOR, not Isaac Lab's ideal one. Same swap the sim makes in
    # custom_rl_env.py, from the same module, so the policy trains and plays on one robot.
    # Takes the leg group's own joint regexes, so the arm's drives below are untouched.
    if measured_actuator:
        apply_go2_actuator(cfg)

    # The Go2's own leg actuator group is scoped to the leg joints by name, so adding the
    # arm's alongside it claims no joint twice.
    cfg.actuators = dict(cfg.actuators)
    cfg.actuators["d1_arm"] = ImplicitActuatorCfg(
        joint_names_expr=["Joint[1-6]"],
        stiffness=ARM_BASE_STIFFNESS,
        damping=ARM_BASE_DAMPING,
    )
    cfg.actuators["d1_gripper"] = ImplicitActuatorCfg(
        joint_names_expr=["Joint7_.*"],
        stiffness=GRIPPER_BASE_STIFFNESS,
        damping=GRIPPER_BASE_DAMPING,
    )
    print(f"[weld] training robot: {weld.usd_path} (arm {arm_mass_kg:.3f} kg, mass scale {weld.arm_mass_scale:.3f})")
    return cfg


def scope_to_legs(env_cfg) -> None:
    """Point the action and the two joint-space observations at the legs only.

    The reward terms are scoped where they are defined (rough_env_cfg.py), because three of
    Spot's penalties ignore `joint_ids` entirely and need the local scoped versions in
    mdp/rewards.py rather than a parameter change.
    """
    env_cfg.actions.joint_pos.joint_names = LEG_JOINTS
    for term in ("joint_pos", "joint_vel"):
        obs_term = getattr(env_cfg.observations.policy, term)
        obs_term.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)}
