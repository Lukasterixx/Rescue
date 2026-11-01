# Unitree Go2-W: the model

The wheeled Go2, as the rescue sim loads it (`--robot go2w`, the default). This folder holds only Unitree's model;
`../go2w.py` turns it into the sim's robot (the USD, the drives, the standing pose), and
[`../rescue_sim/README.md`](../rescue_sim/README.md) says how the sim runs it.

`go2w_description.urdf` and the meshes in `dae/` are Unitree's `robots/go2w_description` from
[unitree_ros](https://github.com/unitreerobotics/unitree_ros), byte-identical with commit `3ae893d` (2026-10-30),
under its BSD 3-clause licence (`LICENCE`). `foot.dae`, which the URDF does not use, is left out.

## What it is

| | |
| --- | --- |
| Trunk | The Go2's: base link 6.921 kg, hips at (±0.1934, ±0.0465) m. The D1 welds on at the Go2's mount |
| Legs | Hip, thigh, calf, named and limited as the Go2's (hip ±60°, front thigh −90° to 200°, rear −30° to 260°, calf −156° to −48°); thigh 0.213 m |
| Below the knee | A 0.646 kg hub motor and a wheel on a continuous joint (`*_foot_joint`, about +y) 0.2264 m below the knee, where the Go2's foot is 0.213 m below it |
| Wheels | 0.172 m across, 52 mm wide, outboard of the calf; 0.52 kg each |
| Mass | 19.5 kg bare (each leg 3.15 kg, the Go2's 2.0); 22.7 kg with the D1 |
| Drives (URDF) | Hip and thigh 23.7 N·m, 30.1 rad/s; calf 35.55 N·m, 20.07 rad/s; wheels 23.7 N·m, 30.1 rad/s |

## What the sim changes, and why

Nothing in this folder is edited. `go2w.build_go2w_usd` converts a copy of the URDF in which:

- the meshes are found here, so no ROS package path is needed;
- each wheel collides as a cylinder the size of its tyre. Its mesh became a convex hull that stood the axles
  78–81 mm off the ground instead of 86;
- every other cylinder collides as a capsule as long as it (or a sphere, where it is shorter than it is wide).
  Cylinders are convex meshes on the GPU, and with them a 4096-robot training step took 468 ms against the Go2's 344;
  with capsules it took 281.

The legs are driven as the Go2's are in this sim (Unitree's measured Go2 HV curve, 20.2 N·m driving and 23.4 braking,
kp 25, kd 0.5), not at the URDF's 35.55 N·m calf. That is how Unitree's own Isaac Lab model of the Go2-W
(unitree_rl_lab, `UNITREE_GO2W_CFG`) drives every leg joint, and how Isaac Lab drives the Go2's: the knee is
weaker here than the URDF says it is. The wheels are velocity-driven at kd 0.5 under 23.5 N·m and 30 rad/s, as in
unitree_rl_lab.

The converted USD is cached in `generated/` (gitignored), keyed by a hash of the URDF, the meshes and the converter's
settings.

Every load warns that `/Robot/Head_upper/visuals` and `/Robot/Head_lower/visuals` are unresolved references. That
is expected: the URDF gives the two head links collision shapes only (the trunk's mesh draws the head), and the
converter references a visual for every link anyway. Nothing is missing from the robot.
