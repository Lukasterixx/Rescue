# The rescue sim

This is the Go2 with its D1 arm and wrist RealSense, in the RoboCup Rescue competition's lanes, plus D1Training's cup demo. It is the only sim in this repository.

```bash
cd ~/Rescue/Isaac/go2_omniverse
./run_sim.sh                               # windowed, starting on the Shifty Gravel lane
./run_sim.sh --level cup                   # start in the cup demo
./run_sim.sh --level stairs                # clear stairs and pallet climb
./run_sim.sh --level stair_debris          # debris stairs, red braces and pallet climb
./run_sim.sh --level ramps_obstacles       # sloped ramps with rotating slip disks and pinch points
./run_sim.sh --headless --smoke-steps 800  # load every level, lie down, stand up and walk, then exit
./run_sim.sh --policy policies/go2_policies_2026-09-18/rough  # walk with another policy (see Walking policies)
./run_sim.sh --help                        # every option
```

The VIP-Rescue website's Dev tab launches the same `run_sim.sh`.

## Levels

The **Rescue sim** window opens as a tab beside the Stage panel, with a row for each arena. An arena with one level is one button. An arena with settings has its name over a button for each: **Flat**, **Slopes 15°** and **Obstacles** for the K-Rails, stepfields and ramps (the guide's Prelims, Prelims/Semis and Semis/Finals columns), Flat and Slopes 15° for Center in Alleys, and Clear and Debris for the stairs. The loaded level's button reads `> `; hovering a button shows the level's full title, what it is and its key. The list scrolls when the dock is short.

| Arena | Setting | Key | What it is |
| --- | --- | --- | --- |
| Shifty Gravel | | `gravel` | Framed OSB floors with loose gravel between rails |
| Diagonal K-Rails | Flat | `krails` | Diagonal rails across 1.2 m panels |
| | Slopes 15° | `krails_slopes` | The same, with the centre floors tilted 15° in opposite directions |
| | Obstacles | `krails_obstacles` | The slopes, with the rails layered to 20 cm and four red pinch points on the railings |
| K-Rail Square | | `square` | The 2.4 m practice square, with the Linear Align/Inspect tasks |
| Half-Cubic Stepfields | Flat | `stepfields` | Diagonal zigzag ridges of 30 cm plateaus with 15 cm ones on every side |
| | Slopes 15° | `stepfields_slopes` | The same, on the opposing 15° centre floors |
| | Obstacles | `stepfields_obstacles` | The slopes, with four pinch points |
| Pitch/Roll Ramps | Flat | `ramps` | 15 cm ramps, peaks and valleys alternating |
| | Slopes 15° | `ramps_slopes` | The same, on the opposing 15° centre floors |
| | Obstacles | `ramps_obstacles` | The slopes, with a rotating slip disk on every ramp and four pinch points |
| Center in Alleys | Flat | `alleys` | Three dividers with doorways, alternating sides |
| | Slopes 15° | `alleys_slopes` | The same, with both far floors rising 15°, so the hallways slope sideways |
| Pallets & Pipes | | `pallets` | Grid pallets, two cells stacked, with pipes against the raised faces |
| Push/Pull Doors | | `doors` | A sprung 90 cm door on a hinge, with steps around it |
| Avoid Holes/Posts | | `avoid` | Ten purchased pallets in a meander, with loose posts to avoid |
| Stairs \| Pallet Climb | Clear | `stairs` | A clear 45° stair to a landing, pallet climb, supported side rails and two belay frames |
| | Debris | `stair_debris` | The same arena with yellow/orange/red beams leaning from treads to side supports, plus red diagonals crossing the path above and below the landing |
| Search & Map Maze | | `maze` | The guide's example maze under a blackout tarp |
| Cup demo | | `cup` | D1Training's pick scene: the Go2 lying down, a 55 × 100 mm mug 42 cm ahead |

All but the cup demo are the competition's lanes from `../competition/`, whose README gives the fabrication details, what is our own reading of the guide, and the options that change them (`--layered-k-rail-height`, `--stair-angle`, and others). The sloped and obstacle copies stand at the far end of the hall, past the maze, but share their arena's row. `--difficulty slopes` now tilts only gravel and pallets, which have no sloped level of their own. A lane added there becomes a level here.

Both stair variants are available in the same session. `--stair-angle` adjusts both; `--stair-debris 0–3`
sets the debris copy's difficulty (default 3). The clear copy always has no debris or red braces.
Debris is fixed collision geometry; the unloaded belay ropes are visual only.

Everything is built into one stage when the sim starts, so loading a level is a teleport, not a rebuild. A load does the following:
- puts the robot on the level's start, with velocities zeroed;
- puts the legs in the level's posture;
- folds the arm back to its rest;
- puts the gravel, the avoid lane's posts and the ramps' slip disks back where they started;
- re-zeroes odometry;
- clears the walking policy's history.

Nav2 and the behaviour tree are not reset; they can send commands straight away.

The window also has these buttons:
- **Reset level** (Home or R) reloads the current level.
- **Lie down / Stand up** (L) ramps the legs over 1.5 s, as a Go2 does on command. The walking policy has the legs only while the robot is standing. The cup demo refuses it: the robot stays lying there and holds still, as in D1Training's pick scene.
- **New cup position** (cup demo only) moves the cup somewhere else in D1Training's band. That band is 0.36–0.44 m ahead and ±0.10 m to the side, with the handle within 45° of pointing straight at or away from the robot.
- **Wrist light** turns a lamp beside the wrist camera on or off (`--wrist-light` starts with it on). The maze's tarp leaves its inside dark, so the camera needs it there. D1Training's camera has no lamp, so it starts off, and the images are D1Training's until it is turned on.

Levels have no keys: there are twenty, too many to step through, so the window's buttons (or `/sim/level`) pick them. Isaac Sim already uses F2 (rename), F7 (hide the whole UI), F10 (screenshot) and F11 (full screen) anyway.

Driving keys:
- **W A S D Q E:** drive the robot, as does `robot0/cmd_vel`.
- **T:** toggle the lidar points.

Click the viewport first so it has keyboard focus.

Scripts can do everything the window does through `/sim/level` (std_msgs/String). Send one of these:
- a level's key, from the table above;
- `reset`, `new_cup`, `lie` or `stand`;
- `light_on` or `light_off`.

### The sim's API, for scripts that test a behaviour

Four more topics let a script reset the robot, start it somewhere else, and judge what it did. They are ground truth
that the robot doesn't have, so use them to judge a behaviour and never to drive one.

| Topic | Type | |
| --- | --- | --- |
| `/sim/teleport` | geometry_msgs/PoseStamped, in | Reloads the current level as `reset` does, but starts the robot at this pose. `frame_id` is `arena`, the level's own frame (the default, also for `""`), or `world`. The arena frame is a lane's origin with the world's axes; for the K-Rail Square that is its centre, +x east, +y north. Only the heading of the orientation is used. A z of 0 means standing at the spawn's height; any other z is the trunk's height. Odometry is re-zeroed where it puts the robot. |
| `/sim/status` | std_msgs/String, JSON, latched | Sent on every change and once a second of sim time. Fields: `level`, `title`, `loads` (counts every load, reset and teleport, so a script can tell its own has happened), `sim_time`, `posture`, `walking`, `fallen` and `fall` (since the last load), `origin` (the arena frame in the world) and `start` (where the last load put the trunk: `x`, `y`, `z`, `yaw_deg` in the world). |
| `/sim/robot_pose` | geometry_msgs/PoseStamped | The trunk's pose in the world, every step. |
| `/sim/cup_pose` | geometry_msgs/PoseStamped | The cup's pose in the world (below). |

A fall is what `training/measure_square.py` counts as one: the trunk tipped past 60° or touching anything. Only
falls while the walking policy has the legs count.

```bash
ros2 topic pub --once /sim/level std_msgs/String "data: square"            # the K-Rail Square, robot on the START pad
ros2 topic pub --once /sim/teleport geometry_msgs/PoseStamped \
  "{header: {frame_id: arena}, pose: {position: {x: -1.8, y: -1.15}, orientation: {z: 0.7071, w: 0.7071}}}"
ros2 topic echo --once /sim/status
```

VIP-Rescue's `go2_control_cpp/test/krail_square/sim_trials.py` uses this API to run the K-Rail Square tree trial after
trial.

**The viewport is yours.** Nothing moves it except one look at each level as it loads. To follow the robot, set Follow Mode to "Asset Root" in the IsaacLab tab's Viewer Settings. The old chase camera is gone.

## Walking policies

By default (since 2026-10-12) the legs are walked by the exported K-Rail Square policy, since 2026-10-13 `policies/go2_krail_2026-10-13` (`sim.py`, `DEFAULT_POLICY`). `--policy DIR` walks them with another exported policy; `DIR` holds `policy.pt` (TorchScript) and `params/deploy.yaml`, and is absolute or relative to this directory. `--policy agent_cfg` walks them with the rsl_rl checkpoint `agent_cfg.py` names, the 2026-09-15 flat policy, which was the default until then; that file's header has its measurements and its own rollbacks.

```bash
./run_sim.sh --level square                                    # the default: 741 wide, steps, K-rails, the K-Rail Square
./run_sim.sh --policy policies/go2_krail_2026-10-12            # the same policy before it learnt to stop on four feet
./run_sim.sh --policy agent_cfg                                # the 2026-09-15 flat policy, 235 wide
./run_sim.sh --policy policies/go2_policies_2026-09-18/flat    # 45 wide: IMU, joints, command, last action
./run_sim.sh --policy policies/go2_policies_2026-09-18/rough   # 232 wide: the same, plus a camera-masked height scan
```

`policies/go2_krail_2026-10-12/` was trained in this repository for the K-Rail Square on this sim's own robot
(`../training/README.md`, "The K-Rail Square policy"). It steps onto and off decks and over K-rails, and went through
the square 20 times in 20 (its README has the measurements, taken in this sim's environment). The 2026-09-18
bundles reached the square's deck in none of 10 trials each; the `agent_cfg` policy has not been put through it.
It reads the stock 48 terms, `base_lin_vel` included, and a full height scan at 5 cm (no camera mask).

`policies/go2_krail_2026-10-13/` is that policy fine-tuned to carry its weight on all four feet (`../training/README.md`,
"The stance fine-tune"), with the same contract. The 2026-10-12 one put 74-86 % of the weight on its front feet
standing after a walk, and every time it was stopped from a forward walk (letting go of W does that, from 1.0 m/s) it
lifted both rear feet for 0.3-0.5 s, nose down 10-12 deg and up to 29. This one stands at 51-56 % front, and 1-2 of
16 robots lifted both rear feet briefly when stopped from 0.3-0.5 m/s and none from faster or other directions, nose
down 3-5 deg. It still went through the square 20 times in 20, and 10 in 10 with 20 cm rails (8 before). It
overshoots forward commands a little more (110 % at 0.3 m/s, 102 % at 1.0) and backs up at 87 % (92 before). Its
README has the measurements.

`policies/go2_policies_2026-09-18/` is the Go2_RL_workflow project's export of 2026-09-18, exactly as it came. Its README says what each policy was trained on and what it expects. Neither has been validated on hardware. The rough one reached terrain level 6.12 of 8 (about 13 cm cubes) and, by that README, struggles above about 15 cm.

The observation is built from the policy's own contract (`exported_policy.py`, `env_cfg.apply_exported_policy`), not from the sim's 235-wide layout:
- **The terms** come in the contract's `observation_order`, each clipped and then scaled as recorded. The flat policy scales the gyro by 0.2 and the joint velocities by 0.05. The rough one scales nothing. Neither reads `base_lin_vel`.
- **The velocity command** is clamped to the envelope it trained in: ±1.5 on each axis for flat, ±1.0 for rough. The keys' 1.0 is inside both. Commands from `robot0/cmd_vel` are clamped the same way.
- **The rough policy's height scan** is moved to where its training had it, centred 0.6 m ahead (from `params/env.yaml`). It is read through a camera mask: cells a forward D435i could not see, and rays that hit nothing, read the contract's `unobserved_value` (0.0). The mask's code is not in the bundle, so it is rebuilt from the bundle README's numbers: a camera at (0.28, 0, 0.10) m pitched 30° down, 89.44° × 58.24°, 0.3–3.0 m. On flat ground it leaves 97 of 187 cells visible, as that README says. Walls are not removed from this scan, as they are for the `agent_cfg` policy, because this policy never trained with that.
- **No action clip** from `agent_cfg.py`: neither policy trained with one. The flat policy's own ±100 sits on its action term.

Before the policy gets the legs, the sim checks that it runs them as trained. If anything differs, it stops and lists what:
- the leg terms in Isaac Lab order: the four hips (FL, FR, RL, RR), then the thighs, then the calves;
- the default pose;
- kp 25 and kd 0.5;
- 50 Hz;
- the observation width;
- the action scale.

The motor is already the one both trained with, Unitree's Go2 HV curve (`go2_actuators.py`). The startup log's `[policy]` lines say what was loaded.

These differ from their training:
- **The arm.** Both trained on the bare Go2; here the D1 adds 3.15 kg on the trunk. The `agent_cfg` policy also trained without it and walks the welded robot well.
- **Actuator delay.** The flat policy trained with 0–4 physics steps of random delay. The sim has none, which is inside that range.
- **Noise.** Training added observation noise; the sim adds none, as for its own policy.
- **The rough policy's mask** is our reading of that README in two places: the camera turns with the whole trunk (roll and pitch too), and its range is depth along the optical axis. On flat ground either reading gives the same 97 cells; on rough ground the edges of the view can differ.

Friction (0.8 static, 0.6 dynamic) is inside both policies' training ranges.

## The arm

**Nothing in the sim drives the arm.** It starts folded at the rest pose D1Training measured on the real D1 (F-023). Joint2 and Joint3 lie on their limits: the upper arm is laid back along the dog and the forearm is folded over it. The arm stays there until something tells it otherwise.

Anything that wants the arm to move talks to it as it would to the real arm, over the D1's own CycloneDDS protocol on domain 0:
- `rt/arm_Command` in;
- `current_servo_angle` and `rt/arm_Feedback` out.

In the VIP-Rescue stack that is `maps/arm_bridge.py --sim`, the bridge the robot runs in its vip-arm container. **The sim starts it for you** (`bridge.py`), with the team's D1 driver, and stops it when the sim exits. It listens on 127.0.0.1:8084, as on the robot. The behaviour tree and the website's arm panel reach the arm through it:
- The cup pick's nodes and the wall scanner use it.
- `ArmToPose` and `ArmStow` use it too.
- Their inverse kinematics runs in C++ (`go2_control_cpp/src/d1_arm`).

The sim finds the bridge under `$VIP_RESCUE_ROOT` (default `~/VIP-Rescue`), with the D1 driver beside it in `maps/d1_driver`. A VIP-Rescue checkout from before the driver moved there (when it was the `Docker/unitree-d1-control` submodule) is refused, with the reason. If a bridge is already listening on 8084, the sim leaves it to serve the arm and starts none. `--arm-bridge off` starts none; `--arm-bridge PATH` starts another script. The bridge logs to `rescue_sim/generated/arm_bridge.log`, and it exits with the sim, even when the window is closed.

The sim-side IK controller and `/arm_commands` are gone.

What the arm does with a command is D1Training's model of the real one (`d1_model.py`, `d1_arm.py`). The parity tests hold it to D1Training's own numbers:

| | The real D1, as D1Training measured it | Here |
| --- | --- | --- |
| Drives | Torques 3.3 N·m (Joint1), 3.33 (Joint2, Joint3: Unitree's D1-550 URDF, not measured, F-137) and 1.7 (the wrist), speed ceilings 1.21–1.29 rad/s (F-033) | Force drives at 4000/400 with those limits (D1Training's `d1_servo`) |
| Motion | The firmware plans a trapezoid to each setpoint: 10 ms dead time, 15.5 rad/s² up, 17.4 down, and each new setpoint restarts from rest (F-045) | The same planner, run every physics step, with velocity feedforward |
| Commands | Streamed at 10 Hz | Passed to the planner at most every 100 ms of sim time, latest first |
| Feedback | Every 111 ms, 0.1° steps (F-020) | Every 6 policy steps (120 ms, D1Training's discretisation), quantised to 0.1° |
| Wire units | Servos 0 and 3 turn against Joint1 and Joint4 (F-030, F-034). The gripper runs from −19.8 (pads touching) to 50.2 (open) (F-063) | The same. The fingers may shut 7.6 mm past the URDF's stop, so the pads meet at 2 mm, as in D1Training's pick |
| funcode 2 mode 1 | Slews at a fifth of mode 0's speed (F-031) | The same, speed only |
| Release (funcode 5 mode 0) | The drives go limp and the arm falls (F-028) | The same; the next motion command re-energises it from where it fell |
| Enable, power off | Enable is implicit; power off is ignored (F-022) | The same |
| Weld, mass | 3.152 kg bolted on the rails, holes 130 mm apart along them and 58 mm across, at (-0.019, 0, 0.0855) on the trunk, on the base measured on the arm (`d1_arm/measured_base.py`: 126 mm across, Joint1 at 76 mm); above Joint1, Unitree's D1-550 CAD lengths (F-138) | The same (`arm_weld.py`, from D1Training's `weld.py`; `d1_arm/` byte-identical with D1Training's); 8/4 solver iterations, self-collisions on. The Go2 EDU's expansion dock and its two rails are drawn on the trunk, visual only (`dock.py`) |

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
| `/glim_rosnode/points` | The RTX lidar, 50 scans a second, in `livox_frame`, with intensity. Each scan is traced in the world and put back in `livox_frame` at the pose the TF gives for its stamp, so a turning robot's walls stay where they are in `odom`, as P2Dingo's sim does. Isaac's own cloud was stamped 0.08 s ahead of the TF and swung the walls round 4° in a 1.25 rad/s turn. `--no-world-lidar` puts it back |
| `/livox/imu` | Orientation, angular velocity and specific force at the lidar, at the policy rate. It is computed, because the old native IMU sensor was never created (Isaac logs "Could not create Imu sensor prim" and carries on) |
| `robot0/cmd_vel` | Velocity command, while standing |
| `/sim/level` | See [Levels](#levels) |
| `/sim/teleport`, `/sim/status`, `/sim/robot_pose` | See [The sim's API](#the-sims-api-for-scripts-that-test-a-behaviour) |
| `/sim/cup_pose` | Ground truth: the cup's pose in the world. For judging a behaviour, never for driving one |

## Speed

The sim runs at **0.54× real time headless** with dynamic gravel on an RTX 4080, and slower in a window. It never runs faster than real time (`--no-realtime` lifts that).

`--profile` prints where each step's time goes. Most of it is `env.step`: 30 ms for four physics steps and the renders of the lidar and wrist camera.

Slow is safe for the arm. Its 10 Hz command hold counts sim time, and the VIP-Rescue arm runtime uses sim time when `use_sim_time` is set. The arm therefore moves in sim time as it would in real time.

## Checked, and not

**Checked on 2026-09-18:**
- **Unit tests.** `python -m unittest discover -s rescue_sim/tests -t .` runs 53 of them (with the conda python, which has torch): the firmware through its wire protocol, the levels and the window's rows, the posture ramp, starting and stopping the arm bridge, and the exported policies' contracts, camera mask and refusals. They include parity with D1Training on fixed inputs (`tests/fixtures/d1training.json`, written by `tests/make_fixtures.py`, last from D1Training 054983a-dirty on 2026-10-11): the planner's trajectories, the calibration's camera model, the mount, the depth noise and the case registration.
- **Smoke run with all twenty levels.** `--smoke-steps 1100 --arm-bridge off`, headless and again in a window, loaded every level, each within 5 cm of its start, and passed, at 0.57× real time headless. It asks the cup demo to stand up and checks that it stays lying. Then it goes back to the first lane, lies down, stands up and walks. It turns the wrist light on for the maze. `--smoke-shots DIR` saves the wrist camera's image after each load, and in a window the viewport and the whole window too. PhysX cooked every slip disk's hull for the GPU.
- **The level window,** from those window captures: in the default window's narrow dock beside Stage, single-level arenas are full-width buttons and the K-Rails, stepfields, ramps, alleys and stairs rows show all their setting buttons, the loaded one highlighted.
- **The slip disks' physics,** in a probe on the GPU pipeline at the sim's 5 ms step: a disk stays seated, turns when pushed across its radius past its seat's friction, holds against a push along it, and lifts only the bolt's 5 mm. `../competition/README.md` has the numbers.
- **The wrist light in the maze.** From a room under the tarp, the wrist camera's image averaged 55 of 255 with the light off and 102 with it on. The walls and fiducials are dim without it, not black.
- **The cup pick end to end, as the robot runs it.** This is the unchanged C++ cup pick (`cup_pick_launch.py robot:=sim`), through `arm_bridge.py --sim` with the team's D1 driver. It found the cup from the survey look, planned an outside grasp and lifted it **11.9 cm** by `/sim/cup_pose`, in 22.7 s of sim time.
- **`ArmToPose` and `ArmStow`.** They moved the arm to the wall scanner's shape, aimed at 0 and at 1 rad, and folded it back each time.

**Checked on 2026-09-19, the exported policies:**
- **The bundles.** The TorchScript and ONNX exports of both policies agree to 4e-6 on random inputs. At the default pose with a zero command, the largest action is 1.0 (flat) and 1.9 (rough), inside the bundle README's sanity check of about 1–2.
- **Smoke runs.** `--smoke-steps 1100 --arm-bridge off`, headless, once for each policy and once for the sim's own. Every run passed: all twenty levels loaded, and the robot stood up and walked. The smoke run now ends with 3 s of walking at 0.5 m/s on the gravel lane; the robot must stay on its feet, and the run reports how far it got:

  | Policy | Forward in 4.7 s | Sideways | Largest action |
  | --- | --- | --- | --- |
  | The sim's own (`agent_cfg.py`) | 1.53 m | +0.10 m | 5.66 |
  | `go2_policies_2026-09-18/flat` | 1.31 m | −0.04 m | 4.94 |
  | `go2_policies_2026-09-18/rough` | 1.81 m | +0.23 m | 5.89 |

  The time includes getting going from a stand. The runs shared the GPU with another job, at 0.32× real time.
- **The rough policy's scan in the sim.** At the gravel spawn, 0.42 m above the floor, it saw 99 of 187 cells, exactly what the rebuilt mask gives at that height offline. Had the grid not moved 0.6 m ahead, it would have seen 33.

**Checked on 2026-10-12, the arm, the dock and the K-Rail Square policy:**
- **The weld with D1Training's latest arm** (`d1_arm/` byte-identical with D1Training's working tree at 054983a: the
  D1-550 CAD's joint lengths, holes 58 mm across the rail, 3.33 N·m at Joint2 and Joint3). The weld composes into
  one 28-body articulation with the 3.152 kg mass model, and draws the expansion dock and rails on the trunk
  (20 081 triangles, visual only). The unit tests pass (59, with the regenerated D1Training fixture and the new
  bundle's).
- **`policies/go2_krail_2026-10-12` is the default policy from this date** (`sim.py`, `DEFAULT_POLICY`; `--policy
  agent_cfg` for the one before).
- **`policies/go2_krail_2026-10-12` through the K-Rail Square**, in this sim's environment without ROS and the arm's
  DDS link (`../training/measure_square.py`, GPU physics): 20 of 20 routes at 0.5 m/s, 10 of 10 at 0.3 and 0.8,
  10 of 10 with the rails at 15 cm and 8 of 10 at 20 cm, none fell. Its README has the rest.
- **`run_sim.sh` itself.** It starts on this machine now: `env_isaaclab` has cyclonedds 11.0.1, PyPI's only wheel for
  Python 3.11. Headless on the K-Rail Square it runs at 0.58× real time.
- **The sim's API** (`/sim/teleport`, `/sim/status`, `/sim/robot_pose`): unit tests for the arena frame, teleport
  poses and the fall test (63 tests in all), and in use below.
- **VIP-Rescue's K-Rail Square tree in a full mission** (`behavior_trees/krail_square.xml`, through
  `minimal_bt_launch_sim.py tree:=krail_square.xml` and `test/krail_square/sim_trials.py`). The route is the course
  drawing's, from the START pad to the END pad. The wall scanner ran with the arm folded low, pivoting on Joint1. The
  arm was driven through `arm_bridge.py --sim` on a spare port, because the website's bridge on 8084 had no cyclonedds.
  - Robot on the pad's centre: 5 of 5 reached the END pad by every stage (onto the deck, over both K-rails, off the
    deck), in 19.4–19.9 s of sim time, with no falls.
  - Robot up to 5 cm and 5° off the pad's centre: 5 of 5, the same.
  - The arm: Joint1 swept up to −114° and +113°, then stowed at the end.
  - Railings coloured: all of the mapped length (6.7 of their 7.2 m) in 9 runs, and 6.2 m in the tenth (the inside
    of the west railing beside the START pad). By cells, 75–94%: the lidar draws a railing two to four cells thick,
    and the scan paints only what it looks at.
  - An earlier 12 runs (the first with no arm: the bridge it reached could not talk to it) all reached the END pad too.
- **The same with `AlignRoute`** (VIP-Rescue's `krail_square.xml` and `route_mission.xml`). Before walking, the
  robot matches the railings it sees against the square's, from `competition/web_export.py`.
  - The robot was put down by `/sim/teleport`, up to 0.64 m and 55° off the pad's centre (15 of the runs off it, one
    on it as a check).
  - Of 16 such runs, 15 reached the END pad by every stage, with no falls. The other found the right pose but scored
    it just under the 0.45 match it then needed; it now needs 0.3.
  - The alignment was within 4.6 cm and 1.3° of `/sim/robot_pose` every time.
  - Three of the runs were `route_mission.xml`, walking a route made active as the Routes tab makes it.

**Checked on 2026-10-13, the four-footed K-Rail Square policy:**
- **`policies/go2_krail_2026-10-13` is the default policy from this date** (`sim.py`, `DEFAULT_POLICY`;
  `--policy policies/go2_krail_2026-10-12` for the one before). Its deploy contract is the 2026-10-12 one's, line for
  line. The unit tests pass (70, both bundles').
- **Through the K-Rail Square** as on 2026-10-12 (`../training/measure_square.py`, GPU physics): 20 of 20 routes at
  0.5 m/s, 10 of 10 at 0.3 and 0.8, 10 of 10 with the rails at 15 cm and at 20 cm, none fell.
- **Stops and stance on flat ground** (`../training/measure_stance.py`), the figures above. Its README has both
  policies side by side.
- **Not checked: `run_sim.sh` with this policy.** The full-mission runs above walked with the 2026-10-12 one, the
  default when they were made. On the machine this one was trained on, `env_isaaclab` still had no `cyclonedds` on
  2026-10-14, so the sim would not start there.

**Not validated:**
- The gait with the arm's drives and solver settings changed to D1Training's. The walking policy was trained with the stock ones; the smoke run shows it standing and walking, not how well it walks.
- The exported policies anywhere but the gravel lane's start. The rough one's reason to exist is terrain: the stepfields, ramps and pallets are where it should do better than the flat policies, and whether it does is untested.
- The wall scanner in the maze's full mission (the K-Rail Square's is checked above).
- Anything on the real robot.
- Gravel behaviour; see `../competition/README.md`.
- Walking any lane from the stepfields on, including all seven sloped and obstacle copies, or pushing the door. The smoke run only loads them. Whether the walking policy copes with 15° side slopes, 20 cm rails, pinch points or turning disks is unknown.

## Files

| File | |
| --- | --- |
| `sim.py` | Arguments, startup, the loop, the wrist light, the smoke run |
| `env_cfg.py` | The environment: Rescue's walking setup for the legs, D1Training's arm and camera, the cup, the competition terrain |
| `levels.py`, `runtime.py` | The level catalogue and the window's rows, the cup demo and the posture ramp; loading, the window and the keys |
| `d1_model.py` | D1Training's arm constants, planner and sampler, verbatim, and the real arm's wire conventions |
| `d1_arm.py`, `d1_drive.py` | The simulated firmware and its DDS link, and where it meets PhysX |
| `bridge.py` | VIP-Rescue's arm bridge, started and stopped with the sim |
| `realsense.py`, `camera_asset.py`, `cup_asset.py` | The camera model, mount, depth noise and ROS publisher, the case, and the cup, from D1Training |
| `ros_io.py` | The ROS node, the lidar, the IMU |
| `exported_policy.py`, `deploy_contract.py` | `--policy`: an exported policy's bundle, its checks, its height scan and camera mask; the bundle's own contract reader, verbatim |
| `assets/` | The calibration, the mount, the D435 case mesh and the cup model, with their licences |
