# Unitree D1 arm in the sim

The D1-550 replaces the Z1 as the manipulator on the Go2. It is mounted on the
robot's back and follows it, with the RGB camera and flashlight on its
end-effector.

The simulated arm speaks the **real D1's DDS protocol**, so anything that drives
it here drives the hardware. This page is about *using* it; for the protocol
itself and the porting checklist see [`../d1_sdk/README.md`](../d1_sdk/README.md).

## Quick start

All paths below are relative to `Isaac/go2_omniverse/`.

```bash
./run_sim.sh
```

Then, from anywhere (the arm is on the DDS bus, not just inside the sim):

```bash
python d1_sdk/examples/read_arm_state.py     # live joint angles at 10 Hz
```

The arm uses **DDS domain 0**, same as the hardware. If that clashes with
something else on your network, move it:

```bash
python main.py --robot_amount 1 --robot go2 --custom_env maze --d1_domain_id 5
```

Requires `cyclonedds` (already installed in `env_isaaclab`). Its traffic is
independent of the sim's ROS 2 bridge, which runs on FastRTPS.

## Keyboard controls

Arm commands all leave over the D1 API, so these keys exercise the same path the
real arm would. (`R` and `T` are simulator controls, not arm commands.)

| Key | Action |
|---|---|
| `↑` / `↓` | move the IK target ±X (forward / back), 2 cm per press |
| `←` / `→` | move the IK target ±Y (left / right) |
| `1` / `0` | move the IK target ±Z (up / down) |
| `,` / `.` | close / open the gripper — reads as `<` / `>` — 5 mm per press (0–65 mm) |
| `Z` | return to the zero pose (suspends IK — see below) |
| `P` | toggle motor power (e-stop) |
| `R` | reset robot **and** arm *(sim)* |
| `T` | toggle lidar debug draw *(sim)* |

The dog's own `WASD` / `QE` teleop is unchanged.

The target is a Cartesian pose **relative to the robot's base**, starting at
`(0.3, 0.0, 0.4)`. Targets are clamped to the arm's 0.55 m reach (measured from
the arm's own base, 8 cm above the robot root) and kept out of the robot's body.

`Z` suspends the IK loop before homing, because that loop rewrites all six
joints every 100 ms and would otherwise overwrite the command before the arm
moved. Press any arm key to re-engage — it resyncs to wherever the arm actually
is, so nothing snaps.

## Interfaces

### 1. Native D1 protocol (CycloneDDS) — the real one

This is what the hardware exposes. **Not ROS** — the arm never appears in
`ros2 topic list`.

| Topic | Type | Direction |
|---|---|---|
| `rt/arm_Command` | `unitree_arm::msg::dds_::ArmString_` | host → arm |
| `rt/arm_Feedback` | `unitree_arm::msg::dds_::ArmString_` | arm → host, 10 Hz |
| `current_servo_angle` | `unitree_arm::msg::dds_::PubServoInfo_` | arm → host, 10 Hz |

```python
from d1_sdk import D1Arm

arm = D1Arm()                              # the simulated arm
arm.wait_for_state()
arm.set_all_joint_angles([10, -20, 30, 0, 15, 0])   # DEGREES
arm.set_gripper(40.0)                               # MILLIMETRES
arm.poll(); arm.get_joint_angles()
```

Units follow the wire, not the URDF: **degrees** for joints, **millimetres** for
the gripper, and protocol ids are **0-based** (`id 0` = `Joint1`). Those traps
and the full funcode table are in [`../d1_sdk/README.md`](../d1_sdk/README.md).

### 2. ROS 2 — what the sim adds around the arm

The arm itself has no ROS interface; these are the simulator's own topics.

| Topic / frame | Type | Notes |
|---|---|---|
| `/arm_commands` | `geometry_msgs/Pose` | **Cartesian** IK target, relative to the robot base. Position + orientation (wxyz). |
| `robot0/front_cam/rgb` | `sensor_msgs/Image` | EE camera, 848×480, ~87° HFOV, 10 Hz |
| `odom` → `arm_camera_link` | TF | end-effector pose |

`/arm_commands` is a convenience for the sim. It does **not** exist on the real
arm — publishing a pose there feeds the same IK controller the keyboard drives.
To write code that ports, go through `D1Arm` instead.

### 3. How a command actually flows

```
keyboard / /arm_commands            d1_sdk/examples/*, your code
        │                                      │
        ▼                                      │
 D1CartesianController  ── IK @ 10 Hz ─────────┤
        │                                      │
        └────────── D1Arm client ──────────────┘
                          │
                   rt/arm_Command   (JSON over DDS)
                          │
              ┌───────────┴────────────┐
              ▼                        ▼
      D1SimServer (here)          real D1 arm
              │
        PhysX joint targets
```

The IK runs on the host at the arm's real **10 Hz**, joint-space only — the D1
has no Cartesian interface, so this is the only way Cartesian control can work,
in sim or on hardware.

## Geometry

| Protocol id | URDF joint | Type | Range |
|---|---|---|---|
| 0 | `Joint1` | revolute | ±135° (base yaw) |
| 1 | `Joint2` | revolute | ±90° (shoulder) |
| 2 | `Joint3` | revolute | ±90° (elbow) |
| 3 | `Joint4` | revolute | ±135° (wrist roll) |
| 4 | `Joint5` | revolute | ±90° (wrist pitch) |
| 5 | `Joint6` | revolute | ±135° (wrist yaw) |
| 6 | `Joint7_1` + `Joint7_2` | prismatic | jaw opening, commanded 0–65 **mm** |

Reach 550 mm, payload 500 g, 6 DOF + gripper. End-effector body is `Link6`;
the camera and flashlight mount there.

Servo id 6 drives both fingers as one jaw: the commanded opening is split
between them (`mm / 2` each, mirrored). The protocol advertises 65 mm, but this
URDF's fingers travel 30 mm each, so a full-open command saturates at 60 mm of
real jaw.

**`Link6`'s frame at the zero pose is `(w=0.707, x=0, y=0.707, z=0)`, not
identity** — a 90° rotation about Y that points the gripper's approach axis along
base +X. This is the D1's convention and differs from the Z1's. Commanding
identity orientation drives Joint5 into its limit and leaves the IK ~13 cm short.
Teleop composes yaw on top of this rest orientation (`D1_EE_REST_ROT`); do the
same if you publish to `/arm_commands`.

## Known gaps vs. hardware

Verified, not assumed:

- **The arm never goes limp.** `P` / `set_motor_power(False)` is faithful on the
  wire (`power_status` flips, drive gains genuinely reach zero) but the arm does
  not sag: its root is teleported onto the robot every step to keep it mounted,
  which resets the articulation's velocity each tick. Drag teaching can't be
  rehearsed here.
- **A streaming controller owns the gripper.** `funcode 2` always carries
  `angle6`, so the IK loop rewrites the jaw every tick. Send gripper commands
  through the *same* `D1Arm` instance (the `G`/`H` keys do). A second client's
  `set_gripper()` gets overwritten within 100 ms — true of the hardware too.
  Call `D1CartesianController.suspend()` first.
- **Encoder latency isn't in the Jacobian.** `IsaacKinematics` evaluates at the
  live PhysX state rather than the 10 Hz reported angles, so the sim is slightly
  optimistic. A URDF-based backend would model it.

## Files

| Path | What |
|---|---|
| `d1_arm/d1.urdf` | the arm. Patched from the Unitree export: mesh paths made relative, and effort/velocity limits filled in — they shipped as **zero**, which gives PhysX no force limit and collapses the arm. |
| `d1_arm/meshes/` | STL geometry (visual + collision) |
| `d1_arm/d1.usd`, `configuration/` | generated from the URDF on every run; gitignored |
| `d1_sdk/` | the protocol, client, and simulated firmware. No Isaac imports. |
| `d1_ik_controller.py` | Cartesian control on top of the D1 API, and the Isaac kinematics backend |
