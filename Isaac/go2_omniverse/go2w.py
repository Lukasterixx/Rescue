"""The Unitree Go2-W, the wheeled Go2: Unitree's own model of it, made into a USD the sim and training both load.

THE MODEL is `go2w_description/`, Unitree's `go2w_description` (unitree_ros, BSD 3-clause; its README says which
commit). The trunk is the Go2's: the same base link, mass and hip positions, so the D1 welds on at the same mount
(`arm_weld.py`) and the dock draws on it as on the Go2. The legs differ below the knee: the calf carries a hub motor
(0.646 kg) and ends in a wheel, 0.172 m across and 52 mm wide, outboard of the calf, on a continuous joint
(`*_foot_joint`) 0.2264 m below the knee where the Go2's foot is 0.213 m below it. Each leg is 3.15 kg against the
Go2's 2.0, so the bare robot is 19.5 kg; with the D1 it is 22.7.

THE JOINTS are the Go2's twelve, named and ordered as the Go2's (`ISAAC_LAB_LEG_ORDER` in `rescue_sim/exported_policy.py`;
`rescue_sim/sim.py` checks the order before a policy gets the legs), plus four wheels.

THE DRIVES:
* The legs: Unitree's measured Go2 HV curve at kp 25, kd 0.5 (`go2_actuators.apply_go2_actuator`), as on the Go2.
  Unitree's own Isaac Lab model of the Go2-W (unitree_rl_lab, `UNITREE_GO2W_CFG`) gives every leg joint the same
  23.5 N*m motor, as Isaac Lab's Go2 does.
* The wheels: velocity-driven, kd 0.5 N*m per rad/s under 23.5 N*m and 30 rad/s, unitree_rl_lab's settings for
  them (kp 0, kd 0.5, the GO2HV group). They are PhysX drives (implicit), not the explicit motor model the legs
  use: a wheel in the air has 0.0012 kg*m^2 about its axle, and an explicit kd 0.5 at the 5 ms physics step
  over-corrects it (dt*kd/I = 2.1, past the stable 2), so it would chatter whenever a leg steps.

THE STANDING POSE is unitree_rl_lab's for the Go2-W: hips 0, thighs 0.8, calves -1.5. It puts each wheel within a
centimetre of under its hip and the base 0.32 m over the axles, 0.41 m over the ground.

THE USD is converted from the URDF with Isaac Lab's `UrdfConverter` on first use and kept in
`go2w_description/generated/` (rebuilt when the URDF, the meshes or the settings below change). Two edits are made to
a copy of the URDF first:
* its `package://go2w_description/` mesh paths point into this folder, so no ROS package path is needed;
* each wheel collides as a cylinder the size of its tyre (`WHEEL_RADIUS_M`, 52 mm wide, where the mesh has it)
  instead of the tyre's mesh. PhysX made the mesh a convex hull that stood the axles 78-81 mm off the ground, not 86.
* every other cylinder (the hips, the calves' four pieces, the hub motors, the head) collides as a capsule as long
  as the cylinder, or a sphere of its radius where the cylinder is shorter than it is wide. On the GPU a cylinder is
  a convex mesh, and convex shapes against the terrain's triangles are the costly pair: with the URDF's cylinders a
  4096-robot K-rail training step took 468 ms against the Go2's 344 (whose USD has spheres for feet), and 281 with
  these capsules, the wheels still cylinders.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
DESCRIPTION = HERE / "go2w_description"
URDF = DESCRIPTION / "go2w_description.urdf"
GENERATED = DESCRIPTION / "generated"

LEG_JOINTS = [".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]
# The wheels in the legs' order (FL, FR, RL, RR). Named, not matched, so they reach a policy in this order whatever
# order PhysX parses them in.
WHEEL_JOINTS = ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"]
WHEELS = "|".join(WHEEL_JOINTS)
STANDING_LEG_POSE = {".*_hip_joint": 0.0, ".*_thigh_joint": 0.8, ".*_calf_joint": -1.5}
WHEEL_RADIUS_M = 0.086
# The tyre's extent along the axle in the wheel's frame, from its mesh: y 0.0222-0.074 (left; the right is mirrored).
WHEEL_WIDTH_M = 0.0518
WHEEL_OFFSET_M = 0.0481
#: The base over the wheels' axles in the standing pose: 0.213 cos(0.8) + 0.2264 cos(0.7).
STANDING_BASE_OVER_AXLES_M = 0.3216

WHEEL_ACTUATOR_GROUP = "wheels"
WHEEL_DAMPING = 0.5            # N*m per rad/s
WHEEL_EFFORT_LIMIT = 23.5      # N*m
WHEEL_VELOCITY_LIMIT = 30.0    # rad/s, 2.6 m/s at the rim
#: rad/s of wheel speed per unit of policy action, robot_lab's and unitree_rl_lab's Go2-W tasks' 5.0: 1 m/s is an
#: action of 2.3.
WHEEL_ACTION_SCALE = 5.0

# PhysX's per-body speed cap, in deg/s. Isaac Lab's Go2 sets 1000 (17 rad/s), under a wheel's 30 rad/s.
MAX_ANGULAR_VELOCITY_DEG_S = 10000.0

_CONVERTER_SETTINGS = {"fix_base": False, "merge_fixed_joints": True, "collider_type": "convex_hull",
                       "make_instanceable": True, "wheel_collider": "cylinder", "leg_colliders": "capsules", "version": 3}


def _fingerprint() -> str:
    h = hashlib.sha256(json.dumps(_CONVERTER_SETTINGS, sort_keys=True).encode())
    for path in [URDF, *sorted((DESCRIPTION / "dae").iterdir())]:
        h.update(path.name.encode())
        h.update(path.read_bytes())
    return h.hexdigest()[:12]


def _urdf_for_conversion() -> str:
    """The URDF with its meshes found in this folder, cylinders for the wheels' colliders and capsules or spheres for
    the other cylinders (module docstring)."""
    import xml.etree.ElementTree as ET

    root = ET.fromstring(URDF.read_text().replace("package://go2w_description/", f"{DESCRIPTION}/"))
    wheels = 0
    for link in root.findall("link"):
        name = link.get("name")
        if not name.endswith("_foot"):
            for geometry in link.iterfind("collision/geometry"):
                cylinder = geometry.find("cylinder")
                if cylinder is None:
                    continue
                radius, length = float(cylinder.get("radius")), float(cylinder.get("length"))
                geometry.remove(cylinder)
                if length > 2.0 * radius:     # a capsule's length is its straight part; the caps add a radius each
                    ET.SubElement(geometry, "capsule", radius=str(radius), length=f"{length - 2.0 * radius:.6g}")
                else:
                    ET.SubElement(geometry, "sphere", radius=str(radius))
            continue
        side = 1.0 if name[1] == "L" else -1.0
        for collision in link.findall("collision"):
            link.remove(collision)
        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin", rpy=f"{1.5707963267948966} 0 0", xyz=f"0 {side * WHEEL_OFFSET_M} 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "cylinder", radius=str(WHEEL_RADIUS_M), length=str(WHEEL_WIDTH_M))
        wheels += 1
    if wheels != 4:
        raise RuntimeError(f"[go2w] {URDF} has {wheels} *_foot links, not the four wheels")
    return ET.tostring(root, encoding="unicode")


def build_go2w_usd(out_dir: Path | str | None = None) -> str:
    """The Go2-W as a USD, converted from `go2w_description/` if it is not already; returns its path. Must run with the
    app up (the converter runs Kit's URDF importer)."""
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    tag = _fingerprint()
    out = Path(out_dir) if out_dir is not None else GENERATED
    usd = out / tag / "go2w.usd"
    if usd.is_file():
        print(f"[go2w] {usd} (cached)", flush=True)
        return str(usd)
    usd.parent.mkdir(parents=True, exist_ok=True)
    urdf = usd.parent / "go2w_description.urdf"
    urdf.write_text(_urdf_for_conversion())
    cfg = UrdfConverterCfg(
        asset_path=str(urdf), usd_dir=str(usd.parent), usd_file_name=usd.name, force_usd_conversion=True,
        fix_base=_CONVERTER_SETTINGS["fix_base"], merge_fixed_joints=_CONVERTER_SETTINGS["merge_fixed_joints"],
        collider_type=_CONVERTER_SETTINGS["collider_type"], make_instanceable=_CONVERTER_SETTINGS["make_instanceable"],
        self_collision=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force", target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)),
    )
    UrdfConverter(cfg)
    print(f"[go2w] converted {URDF.name} -> {usd}", flush=True)
    return str(usd)


def articulation_cfg(usd_path: str):
    """The Go2-W as Isaac Lab's Go2 is configured (contact sensors, the same rigid-body and solver settings), with the
    legs in a `base_legs` group for `go2_actuators.apply_go2_actuator` to give the measured motor, and the wheels'
    velocity drives."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

    from go2_actuators import GO2_LEG_ACTUATOR_GROUP

    go2 = UNITREE_GO2_CFG
    return ArticulationCfg(
        spawn=go2.spawn.replace(
            usd_path=usd_path,
            rigid_props=go2.spawn.rigid_props.replace(max_angular_velocity=MAX_ANGULAR_VELOCITY_DEG_S)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.45), joint_pos={**STANDING_LEG_POSE, ".*_foot_joint": 0.0}, joint_vel={".*": 0.0}),
        soft_joint_pos_limit_factor=go2.soft_joint_pos_limit_factor,
        actuators={
            # Replaced by the measured motor; the stock Go2's group, so nothing is left out if it is not.
            GO2_LEG_ACTUATOR_GROUP: DCMotorCfg(
                joint_names_expr=list(LEG_JOINTS), effort_limit=23.5, saturation_effort=23.5, velocity_limit=30.0,
                stiffness=25.0, damping=0.5, friction=0.0),
            WHEEL_ACTUATOR_GROUP: ImplicitActuatorCfg(
                joint_names_expr=list(WHEEL_JOINTS), stiffness=0.0, damping=WHEEL_DAMPING,
                effort_limit_sim=WHEEL_EFFORT_LIMIT, velocity_limit_sim=WHEEL_VELOCITY_LIMIT),
        },
    )
