# Unitree D1 arm — SDK and simulated arm

A faithful implementation of the D1's native protocol, plus a simulated arm that
speaks it. The point is that **the same client code drives the simulator and the
real arm**.

```python
from d1_sdk import D1Arm

arm = D1Arm()                          # simulated arm
arm = D1Arm(network_interface="eth0")  # real arm at 192.168.123.100
```

## Install

```bash
pip install cyclonedds        # the only dependency; already in env_isaaclab
```

The arm's DDS traffic is independent of the sim's ROS 2 bridge (which runs on
FastRTPS), and the two coexist. The arm uses **DDS domain 0**, like the real
hardware; `--d1_domain_id` moves it if that collides with something.

Handy while the sim is running:

```bash
python d1_sdk/examples/read_arm_state.py     # live joint angles, 10 Hz
```

## The D1 is not a ROS arm

Unlike the Z1 this replaced, the D1 exposes **no ROS topics, services or
actions**, and there is no `unitree_arm_msgs` package. It is raw **CycloneDDS**
with four IDL types under `unitree_arm::msg::dds_`. This is why the arm never
shows up in `ros2 topic list` — people lose hours to that.

The official "SDK" is a zip of example `.cpp` files. There is no library, no
`D1Arm` class, and no official Python binding. This package supplies the client
Unitree doesn't, modelled on the community [D1Py](https://github.com/omar-mostafa81/D1Py).

## Wire protocol

| Topic | Type | Direction |
|---|---|---|
| `rt/arm_Command` | `ArmString_` | host → arm |
| `rt/arm_Feedback` | `ArmString_` | arm → host, 10 Hz |
| `current_servo_angle` | `PubServoInfo_` | arm → host, 10 Hz |

`current_servo_angle` has **no `rt/` prefix**. That asymmetry is in the real
firmware; do not "fix" it.

Every command is JSON inside a DDS string:

```json
{"seq": 1, "address": 1, "funcode": 2, "data": {"mode": 0, "angle0": 10.0, ...}}
```

| address | funcode | meaning |
|---|---|---|
| 1 | 1 | one joint: `{id, angle, delay_ms}` |
| 1 | 2 | all joints: `{mode, angle0..angle6}` |
| 1 | 4 / 5 | damping, one joint / all |
| 1 | 6 | motor power (e-stop) |
| 1 | 7 | return to zero |
| 2 | 1 / 3 / 4 | state: angles / status / motor status |
| 3 | 1 / 2 | ack: received / executed |

## Traps worth knowing

These each cost real debugging time:

- **Units are degrees**, not radians. The URDF is radians. Convert at the boundary.
- **The gripper is millimetres.** Servo id 6 uses the same `angle` field as the
  joints, but it means 0–65 mm of jaw stroke. Same field, same funcode,
  different unit.
- **Protocol ids are 0-based, the URDF is 1-based.** Protocol `id: 0` is URDF
  `Joint1`. An off-by-one moves the wrong joint, silently.
- **Damping is a 0–80000 range, not a boolean.** 0 frees a joint for drag
  teaching; 80000 locks it. Some community code treats it as on/off.
- **`mode` on funcode 2 is smoothing, not a control mode.** 0 for streamed
  setpoints, 1 for discrete waypoints.
- **The arm acks before it acts.** Clients block on the address-3 acks; a server
  that doesn't emit them will hang its clients.
- **Control is 10 Hz.** It is a bus-servo arm, not a quasi-direct-drive one.
  Don't design for a high-rate loop.
- **Never add `from __future__ import annotations` to `d1_msgs.py`** — it
  stringifies the annotations and CycloneDDS cannot resolve the IDL types.

## Cartesian control

The real arm has **no Cartesian interface** — joint angles in degrees at 10 Hz,
nothing else. So IK is necessarily a host-side loop: read angles, solve, write
angles back. `d1_ik_controller.py` does exactly that, through `D1Arm`, so the
loop is identical against sim and hardware.

### A streaming controller owns the gripper

`funcode 2` always carries `angle0..angle6`, and `angle6` **is** the gripper. So
anything streaming all-angles at 10 Hz rewrites the gripper every tick. Two
consequences, both true of the hardware as well:

- Send gripper commands through **the same `D1Arm` instance** the controller
  uses. `D1Arm` remembers the last stroke you commanded and keeps sending it, so
  `set_gripper()` sticks. The sim's `,` / `.` keys work this way.
- A **second** client calling `set_gripper()` while a controller streams will be
  overwritten within 100 ms. That is not a simulator artefact — two controllers
  fighting over one arm is what the protocol allows. Suspend the streaming
  controller (`D1CartesianController.suspend()`) before driving the arm from
  somewhere else.

The same applies to `return_to_zero`: it sets all six joint targets, and a
streaming controller overwrites them on the next tick. Suspend first.

## Porting to the real arm

Everything Isaac-specific is behind `KinematicsBackend`. Two steps:

1. **Point the client at the arm.** `D1Arm(network_interface="eth0")`. This is
   the equivalent of the C++ SDK's `ChannelFactory::Init(0, "eth0")`, and the
   only change on the command/state path. Don't run `D1SimServer` — the real
   arm is the server.

2. **Swap the kinematics backend.** `IsaacKinematics` reads PhysX. Implement the
   same two methods on top of a URDF kinematics library (pinocchio, PyKDL)
   loading `d1_arm/d1.urdf`:

   ```python
   class PinocchioKinematics(KinematicsBackend):
       def snapshot(self, reported_angles_rad) -> KinSnapshot:
           # FK + Jacobian evaluated AT reported_angles_rad, so the snapshot
           # stays self-consistent -- that is all the solver requires.
           ...
       def to_world(self, rel_pos, rel_quat):
           # On a fixed arm this is identity: targets are already arm-relative.
           ...
   ```

   `snapshot()` must return the EE pose, the Jacobian, **and** the joint angles
   they were evaluated at, as one consistent set. `IsaacKinematics` ignores the
   reported angles and returns live PhysX state, because PhysX evaluates its
   Jacobian at the live configuration; a pinocchio backend should honour them.
   That difference is the one place sim flatters hardware — the simulator does
   not model the arm's 10 Hz encoder latency in the Jacobian.

Then `D1CartesianController` runs unchanged.

## Where the simulator differs from hardware

Two known gaps, both verified rather than assumed:

- **Going limp is not modelled.** `set_motor_power(False)` and
  `set_all_damping(0)` do the right thing on the wire — `power_status` flips and
  the drive gains genuinely drop to zero — but the arm does not sag. Its root is
  teleported onto the quadruped every step to keep it mounted, which resets the
  articulation's velocity each tick, so the joints never accumulate any falling
  motion (measured joint velocity stays at ~1e-4 rad/s either way). Drag
  teaching therefore cannot be rehearsed in sim.
- **Encoder latency is not in the Jacobian.** See the porting notes above:
  `IsaacKinematics` evaluates at the live PhysX state rather than the 10 Hz
  reported angles.

## Simulator bridge

`D1SimServer` holds no simulator state. The sim loop pushes measured angles in
and pulls commanded targets out:

```python
server.set_measured(angles_deg, gripper_mm)   # simulated encoders
targets_deg, gripper_mm = server.get_targets()  # what the host asked for
```

Because it is real DDS, **any** process on the bus can drive the simulated arm —
including a stock D1 client that has no idea it isn't talking to hardware.
