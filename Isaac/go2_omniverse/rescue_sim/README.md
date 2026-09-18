# The rescue sim

This is the Go2 with its D1 arm and wrist RealSense, in the RoboCup Rescue competition's lanes, plus D1Training's cup demo. It is the only sim in this repository.

```bash
cd ~/Rescue/Isaac/go2_omniverse
./run_sim.sh                               # windowed, starting on the Shifty Gravel lane
./run_sim.sh --level cup                   # start in the cup demo
./run_sim.sh --headless --smoke-steps 800  # load every level, lie down and stand up, then exit
./run_sim.sh --help                        # every option
```

The VIP-Rescue website's Dev tab launches the same `run_sim.sh`.

## Levels

The **Rescue sim** window has one button per level. It opens as a tab beside the Stage panel.

| Level | Key | What it is |
| --- | --- | --- |
| Shifty Gravel | `gravel` | Framed OSB floors with loose gravel between rails |
| Diagonal K-Rails | `krails` | Diagonal rails across 1.2 m panels |
| K-Rail Square | `square` | The 2.4 m practice square, with the Linear Align/Inspect tasks |
| Half-Cubic Stepfields | `stepfields` | Diagonal zigzag ridges of 30 cm plateaus with 15 cm ones on every side |
| Pitch/Roll Ramps | `ramps` | 15 cm ramps, peaks and valleys alternating |
| Center in Alleys | `alleys` | Three dividers with doorways, alternating sides |
| Pallets & Pipes | `pallets` | Grid pallets, two cells stacked, with pipes against the raised faces |
| Push/Pull Doors | `doors` | A sprung 90 cm door on a hinge, with steps around it |
| Avoid Holes/Posts | `avoid` | Ten purchased pallets in a meander, with loose posts to avoid |
| Stair Debris \| Pallet Climb | `stairs` | A 45° stair to a landing, and a climb over stacked pallets |
| Search & Map Maze | `maze` | The guide's example maze under a blackout tarp |
| Cup demo | `cup` | D1Training's pick scene: the Go2 lying down, a 55 × 100 mm mug 42 cm ahead |

The first eleven are the competition's lanes from `../competition/`, whose README gives the fabrication details and the options that change them (`--difficulty slopes`, `--stair-angle`, and others). A lane added there becomes a level here.

Everything is built into one stage when the sim starts, so loading a level is a teleport, not a rebuild. A load does the following:
- puts the robot on the level's start, with velocities zeroed;
- puts the legs in the level's posture;
- folds the arm back to its rest;
- puts the gravel and the avoid lane's posts back where they started;
- re-zeroes odometry;
- clears the walking policy's history.

Nav2 and the behaviour tree are not reset; they can send commands straight away.

The window also has these buttons:
- **Reset level** (Home or R) reloads the current level. Page Down and Page Up load the next and previous level.
- **Lie down / Stand up** (L) ramps the legs over 1.5 s, as a Go2 does on command. The walking policy has the legs only while the robot is standing. The cup demo refuses it: the robot stays lying there and holds still, as in D1Training's pick scene.
- **New cup position** (cup demo only) moves the cup somewhere else in D1Training's band. That band is 0.36–0.44 m ahead and ±0.10 m to the side, with the handle within 45° of pointing straight at or away from the robot.
- **Wrist light** turns a lamp beside the wrist camera on or off (`--wrist-light` starts with it on). The maze's tarp leaves its inside dark, so the camera needs it there. D1Training's camera has no lamp, so it starts off, and the images are D1Training's until it is turned on.

Levels have no F-keys. Isaac Sim already uses F2 (rename), F7 (hide the whole UI), F10 (screenshot) and F11 (full screen).

Driving keys:
- **W A S D Q E:** drive the robot, as does `robot0/cmd_vel`.
- **T:** toggle the lidar points.

Click the viewport first so it has keyboard focus.

Scripts can do everything the window does through `/sim/level` (std_msgs/String). Send one of these:
- a level's key, from the table above;
- `reset`, `new_cup`, `lie` or `stand`;
- `light_on` or `light_off`.

**The viewport is yours.** Nothing moves it except one look at each level as it loads. To follow the robot, set Follow Mode to "Asset Root" in the IsaacLab tab's Viewer Settings. The old chase camera is gone.

## The arm

**Nothing in the sim drives the arm.** It starts folded at the rest pose D1Training measured on the real D1 (F-023). Joint2 and Joint3 lie on their limits: the upper arm is laid back along the dog and the forearm is folded over it. The arm stays there until something tells it otherwise.

Anything that wants the arm to move talks to it as it would to the real arm, over the D1's own CycloneDDS protocol on domain 0:
- `rt/arm_Command` in;
- `current_servo_angle` and `rt/arm_Feedback` out.

In the VIP-Rescue stack that is `maps/arm_bridge.py --sim`, the bridge the robot runs in its vip-arm container. **The sim starts it for you** (`bridge.py`), with the team's D1 driver, and stops it when the sim exits. It listens on 127.0.0.1:8084, as on the robot. The behaviour tree and the website's arm panel reach the arm through it:
- The cup pick's nodes and the wall scanner use it.
- `ArmToPose` and `ArmStow` use it too.
- Their inverse kinematics runs in C++ (`go2_control_cpp/src/d1_arm`).

The sim finds the bridge under `$VIP_RESCUE_ROOT` (default `~/VIP-Rescue`) and the driver in that repo's `Docker/unitree-d1-control` submodule (`git submodule update --init Docker/unitree-d1-control` once; `$D1_DRIVER_ROOT` points elsewhere). If a bridge is already listening on 8084, the sim leaves it to serve the arm and starts none. `--arm-bridge off` starts none; `--arm-bridge PATH` starts another script. The bridge logs to `rescue_sim/generated/arm_bridge.log`, and it exits with the sim, even when the window is closed.

The sim-side IK controller and `/arm_commands` are gone.

What the arm does with a command is D1Training's model of the real one (`d1_model.py`, `d1_arm.py`). The parity tests hold it to D1Training's own numbers:

| | The real D1, as D1Training measured it | Here |
| --- | --- | --- |
| Drives | Published torques 3.3 / 1.7 N·m, speed ceilings 1.21–1.29 rad/s (F-033) | Force drives at 4000/400 with those limits (D1Training's `d1_servo`) |
| Motion | The firmware plans a trapezoid to each setpoint: 10 ms dead time, 15.5 rad/s² up, 17.4 down, and each new setpoint restarts from rest (F-045) | The same planner, run every physics step, with velocity feedforward |
| Commands | Streamed at 10 Hz | Passed to the planner at most every 100 ms of sim time, latest first |
| Feedback | Every 111 ms, 0.1° steps (F-020) | Every 6 policy steps (120 ms, D1Training's discretisation), quantised to 0.1° |
| Wire units | Servos 0 and 3 turn against Joint1 and Joint4 (F-030, F-034). The gripper runs from −19.8 (pads touching) to 50.2 (open) (F-063) | The same. The fingers may shut 7.6 mm past the URDF's stop, so the pads meet at 2 mm, as in D1Training's pick |
| funcode 2 mode 1 | Slews at a fifth of mode 0's speed (F-031) | The same, speed only |
| Release (funcode 5 mode 0) | The drives go limp and the arm falls (F-028) | The same; the next motion command re-energises it from where it fell |
| Enable, power off | Enable is implicit; power off is ignored (F-022) | The same |
| Weld, mass | 3.152 kg at (0, 0, 0.08) on the trunk | The same (`arm_weld.py`, from D1Training's `weld.py`); 8/4 solver iterations, self-collisions on |

Three things differ from the real arm, on purpose:
- **It starts energised, holding its rest.** The real arm rests limp on its stops until the first motion command. Held, the folded arm cannot flop about while the dog walks.
- **A level load is not undone by a streamed old target.** The bridge keeps repeating the last target it was given. After a load, the arm ignores that repeat and stays folded until it is sent somewhere new.
- **The gripper's middle is assumed.** Units map to travel linearly between the two measured ends, because nobody has measured the jaw between them (F-063). The same assumption holds on the real arm.

## The camera

This is D1Training's wrist RealSense (`realsense.py`, `camera_asset.py`):
- **Intrinsics:** the bench D435i's own calibration (`assets/calibration/`, D1Training F-053), 640 × 480. `--camera d435` gives D1Training's datasheet preset instead, which its simulated picks default to.
- **Mount:** D1Training's `wrist_mount.json` on Link6. It was aligned by eye against the CAD, not measured.
- **Near clip:** 20 mm, which keeps the camera from seeing its own case. The case is Intel's D435 CAD, visual only (`--no-camera-body` hides it).
- **Depth:** perfect rendered depth, zeroed outside 0.154–3.0 m, with Intel's stereo RMS noise inside that range (`--no-depth-noise` turns the noise off).

It is published as the robot's realsense container publishes it:
- colour on `/camera/color/image_raw` (rgb8);
- depth on `/camera/depth/image_rect_raw`;
- a `camera_info` for each;
- 15 Hz by default (`--camera-hz`).

Depth differs from the container in two ways: it is already aligned to the colour pixels, and it is float metres (32FC1, 0 = invalid). The cup pick reads both kinds.

Two renderer limits come with it, as in D1Training:
- **The principal point and pixel shape are not the calibration's.** Omniverse renders square pixels about the image centre. The images therefore use fx = fy = 607.24 at (320, 240), not the calibration's (323.0, 254.3). `camera_info` carries what was rendered (D1Training F-070).
- **The rendered eye sits about 11 mm from where the mount puts it,** constant in Link6 (D1Training F-052, cause unexplained).

`/photo_request_str` and `/record_video_str` save the colour images under `photos/<id>/`.

## Everything else on ROS

| Topic | |
| --- | --- |
| `/clock` | Sim time; everything is stamped with it |
| `/joint_states` | All 20 joints |
| `/odom`, TF `odom → livox_frame`, static `odom → map` | Re-zeroed at startup and on every load. `ros2 param set /go2_driver_node publish_map_odom false` stops them |
| `/glim_rosnode/points` | The RTX lidar, as before |
| `/livox/imu` | Orientation, angular velocity and specific force at the lidar, at the policy rate. It is computed, because the old native IMU sensor was never created (Isaac logs "Could not create Imu sensor prim" and carries on) |
| `robot0/cmd_vel` | Velocity command, while standing |
| `/sim/level` | See [Levels](#levels) |
| `/sim/cup_pose` | Ground truth: the cup's pose in the world. For judging a behaviour, never for driving one |

## Speed

The sim runs at **0.54× real time headless** with dynamic gravel on an RTX 4080, and slower in a window. It never runs faster than real time (`--no-realtime` lifts that).

`--profile` prints where each step's time goes. Most of it is `env.step`: 30 ms for four physics steps and the renders of the lidar and wrist camera.

Slow is safe for the arm. Its 10 Hz command hold counts sim time, and the VIP-Rescue arm runtime uses sim time when `use_sim_time` is set. The arm therefore moves in sim time as it would in real time.

## Checked, and not

**Checked on 2026-09-18:**
- **Unit tests.** `python -m unittest discover -s rescue_sim/tests -t .` runs 38 of them: the firmware through its wire protocol, the levels, the posture ramp, and starting and stopping the arm bridge. They include parity with D1Training on fixed inputs (`tests/fixtures/d1training.json`, written by `tests/make_fixtures.py` from D1Training 5e19028): the planner's trajectories, the calibration's camera model, the mount, the depth noise and the case registration.
- **Smoke run.** `--smoke-steps 800` loads all twelve levels, each within 5 cm of its start. It asks the cup demo to stand up and checks that it stays lying. Then it goes back to the first lane, lies down, stands up and walks. It turns the wrist light on for the maze. `--smoke-shots DIR` saves the wrist camera's image after each load.
- **The wrist light in the maze.** From a room under the tarp, the wrist camera's image averaged 55 of 255 with the light off and 102 with it on. The walls and fiducials are dim without it, not black.
- **The cup pick end to end, as the robot runs it.** This is the unchanged C++ cup pick (`cup_pick_launch.py robot:=sim`), through `arm_bridge.py --sim` with the team's D1 driver. It found the cup from the survey look, planned an outside grasp and lifted it **11.9 cm** by `/sim/cup_pose`, in 22.7 s of sim time.
- **`ArmToPose` and `ArmStow`.** They moved the arm to the wall scanner's shape, aimed at 0 and at 1 rad, and folded it back each time.

**Not validated:**
- The gait with the arm's drives and solver settings changed to D1Training's. The walking policy was trained with the stock ones; the smoke run shows it standing and walking, not how well it walks.
- The wall scanner in a full mission.
- Anything on the real robot.
- Gravel behaviour; see `../competition/README.md`.
- Walking any of the eight newer lanes (stepfields to maze), or pushing the door. The smoke run only loads them.

## Files

| File | |
| --- | --- |
| `sim.py` | Arguments, startup, the loop, the wrist light, the smoke run |
| `env_cfg.py` | The environment: Rescue's walking setup for the legs, D1Training's arm and camera, the cup, the competition terrain |
| `levels.py`, `runtime.py` | The level catalogue, the cup demo and the posture ramp; loading, the window and the keys |
| `d1_model.py` | D1Training's arm constants, planner and sampler, verbatim, and the real arm's wire conventions |
| `d1_arm.py`, `d1_drive.py` | The simulated firmware and its DDS link, and where it meets PhysX |
| `bridge.py` | VIP-Rescue's arm bridge, started and stopped with the sim |
| `realsense.py`, `camera_asset.py`, `cup_asset.py` | The camera model, mount, depth noise and ROS publisher, the case, and the cup, from D1Training |
| `ros_io.py` | The ROS node, the lidar, the IMU |
| `assets/` | The calibration, the mount, the D435 case mesh and the cup model, with their licences |
