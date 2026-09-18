# Unitree D1 arm: the model

The D1-550 on the Go2's back, as the rescue sim loads it. This folder holds only the model. What the arm does with
a command, how to drive it, and how closely it matches the real arm are in
[`../rescue_sim/README.md`](../rescue_sim/README.md#the-arm).

The sim has no controller of its own for the arm. The keyboard IK target, `/arm_commands` and the chase camera
described here before are gone. The arm starts folded at the real D1's measured rest and moves only when something
sends it the real arm's DDS commands: in the VIP-Rescue stack, `maps/arm_bridge.py --sim` behind the behaviour
tree.

## Geometry

| Protocol id | URDF joint | Type | Range |
|---|---|---|---|
| 0 | `Joint1` | revolute | ±135° (base yaw) |
| 1 | `Joint2` | revolute | ±90° (shoulder) |
| 2 | `Joint3` | revolute | ±90° (elbow) |
| 3 | `Joint4` | revolute | ±135° (wrist roll) |
| 4 | `Joint5` | revolute | ±90° (wrist pitch) |
| 5 | `Joint6` | revolute | ±135° (wrist yaw) |
| 6 | `Joint7_1` + `Joint7_2` | prismatic | the jaw, −19.8 (pads touching) to 50.2 (open) on the wire |

Reach 550 mm, payload 500 g, 6 DOF plus gripper. The end-effector body is `Link6`, and the wrist RealSense mounts
there.

The wire follows the real arm, not the URDF:
- Joints are in degrees and protocol ids are 0-based (`id 0` = `Joint1`).
- Servos 0 and 3 turn against `Joint1` and `Joint4`.
- Servo 6 drives both fingers as one jaw.

`rescue_sim/d1_model.py` holds these conventions.

**`Link6`'s frame at the zero pose is `(w=0.707, x=0, y=0.707, z=0)`, not identity.** It is a 90° rotation about Y
that points the gripper's approach axis along base +X. This is the D1's convention and differs from the Z1's. An
identity orientation drives Joint5 into its limit.

## Files

| Path | What |
|---|---|
| `d1.urdf` | The arm. It is patched from the Unitree export: mesh paths are made relative, and the effort/velocity limits are filled in. Those shipped as **zero**, which gives PhysX no force limit and collapses the arm. The rescue sim replaces the limits with D1Training's published torques and speed ceilings. |
| `meshes/` | STL geometry (visual and collision) |
| `d1.usd`, `configuration/` | Generated from the URDF |
