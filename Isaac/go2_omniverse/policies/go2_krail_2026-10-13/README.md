# Go2 + D1 walking policy for the K-Rail Square, on four feet

The K-Rail Square policy (`../go2_krail_2026-10-12`) fine-tuned to carry its weight on all four feet and to stop
without a handstand. Same robot, observation, action and contract; same task
(`../../training/go2_rescue/krail_env_cfg.py`, `Isaac-Velocity-Rescue-KRail-Go2-v0`), with the stance terms that
file's `KRailRewardsCfg` describes. Exported on 2026-10-13 by `../../training/export_krail.py`. The rescue sim's
default policy from that date.

```bash
cd ~/Rescue/Isaac/go2_omniverse
./run_sim.sh --level square          # the default; --policy policies/go2_krail_2026-10-12 for the one before
```

```
policy.pt        TorchScript: (1, 741) observation -> (1, 12) raw action, the policy mean, clamped at +-20
policy.onnx      the same network; input `obs`, output `actions`
params/deploy.yaml   the deploy contract (`rescue_sim/deploy_contract.py`), identical to 2026-10-12's
params/env.yaml      the training run's environment, which the sim reads the height scanner from
params/agent.yaml    the training run's runner config (of the last segment, see below)
params/export.json   source run and checkpoint, sha256 of each file, TorchScript/ONNX agreement
eval/                the measurements below, as JSON
```

The source checkpoint, for fine-tuning, is `../../logs/rsl_rl/go2_krail/2026-10-12_22-51-38_v3e/model_9300.pt`.

## What it expects

The sim checks the legs, gains, control rate and observation width against the contract before the policy gets
the legs, and stops if anything differs.

| | |
|---|---|
| Observation | 741: `base_lin_vel`, `base_ang_vel`, `projected_gravity`, `velocity_commands`, the 12 legs' `joint_pos_rel` and `joint_vel_rel`, `last_action`, then the height scan. Unscaled and unclipped, except the scan at +-1 |
| Height scan | 693 rays, a 1.6 x 1.0 m grid at 5 cm centred on the base, aligned with its yaw, `base_z - hit_z - 0.5`. Every cell, no camera mask |
| Commands | vx +-1.0 m/s, vy +-0.8 m/s, yaw +-1.5 rad/s; the sim clamps to these |
| Action | 12 leg position targets, `default + 0.25 * action`, Isaac Lab leg order; clamped at +-20 inside `policy.pt`, as training clamped it |
| Legs | Unitree's Go2 HV motor curve (`go2_actuators.py`), kp 25, kd 0.5, 50 Hz |

It reads `base_lin_vel`, which the sim provides and a real Go2 cannot measure directly: on the robot it would
need a velocity estimate (GLIM's odometry, say). It has not been run on hardware.

## Measured (2026-10-13), against 2026-10-12

All on GPU physics, the 2026-10-12 policy re-measured beside it where it had not been.

**Stance and stopping** (`../../training/measure_stance.py`, `eval/stance.json`; 16 robots a case on flat ground,
the nominal robot, the command held 4 s and then set to zero, as letting go of the sim's drive keys does):

| | 2026-10-12 | 2026-10-13 |
|---|---|---|
| stopped from 0.3 / 0.5 / 0.8 / 1.0 m/s forward: robots with both rear feet up over 0.1 s | 16 / 16 / 16 / 16 of 16 | 2 / 1 / 0 / 0 of 16 |
| ... both rear feet up together, mean (worst robot) | 0.33-0.46 s (1.06 s) | 0.01-0.05 s (0.22 s) |
| ... highest a rear foot rose | 17-21 cm | 8-11 cm (a step) |
| ... nose down at most, mean (worst robot) | 10-12 deg (29 deg) | 3-5 deg (9 deg) |
| ... centre of mass behind the front feet, least of any robot | -0.7 to +0.5 cm (over them) | 2.6-10 cm |
| stopped from a 0.5 m/s, 0.5 rad/s arc: robots with both rear feet up | 15 of 16 | 0 of 16 |
| front feet's share of the weight, walking forward 0.3 / 0.5 / 0.8 / 1.0 m/s | 83 / 74 / 62 / 56 % | 62 / 54 / 48 / 45 % |
| front feet's share standing, 2 s after a stop | 61-86 % | 51-56 % |
| all four feet down, 2 s after a stop | 19-96 % of the time | 82-100 % |
| feet's distance from the centre line, standing after a stop | 6-10 cm | 15-21 cm |
| base height over its feet, standing | 0.31-0.33 m | 0.29-0.31 m |

Sideways, backward and turning stops lifted both rear feet in at most 6 of 16 robots before and none now.

**K-Rail Square** (`../../training/measure_square.py`: the sim's own environment and hall, without the lidar, the
camera, ROS and the arm's DDS firmware; the course drawing's route, START pad, up the 12.2 cm step, over two K-rails
round the crate, down the step, END pad, driven by a nav2-like driver that turns on the spot and never strafes):

| | 2026-10-12 | 2026-10-13 |
|---|---|---|
| 0.5 m/s, 10 cm rails (the square as built) | 20 of 20, 19.1 s | **20 of 20**, 19.1 s, no falls |
| 0.3 m/s | 10 of 10 | 10 of 10, 27.6 s |
| 0.8 m/s | 10 of 10 | 10 of 10, 15.0 s |
| 0.5 m/s, 15 cm rails (`--k-rail-height 0.15`) | 10 of 10 | 10 of 10 |
| 0.5 m/s, 20 cm rails | 8 of 10 | **10 of 10** |
| along the 0.5 m/s route: speed / turn rate held, where the command was steady | 101.5 / 98 % | 106 / 98 % |
| ... mean absolute error, vx / vy / wz | 0.024 m/s / 0.011 m/s / 0.056 rad/s | 0.033 / 0.021 / 0.051 |

**Held commands on flat ground** (`measure_tracking.py`, `eval/tracking_flat.json`; 16 robots each, 12 s, the first 3
dropped; velocities in the yaw frame):

| command | 2026-10-12 | 2026-10-13 | | command | 2026-10-12 | 2026-10-13 |
|---|---|---|---|---|---|---|
| forward 0.3 m/s | 105 % | 110 % | | turn on the spot 0.5 rad/s | 107 % | 102 % |
| forward 0.5 m/s | 102 % | 106 % | | turn on the spot 1.0 rad/s | 104 % | 99 % |
| forward 0.8 m/s | 100 % | 104 % | | turn on the spot 1.5 rad/s | 102 % | 99 % |
| forward 1.0 m/s | 100 % | 102 % | | turn on the spot -1.0 rad/s | 106 % | 99 % |
| backward 0.5 m/s | 92 % | 87 % | | arc, 0.5 m/s and 0.5 rad/s | 103 % | 106 % |
| left / right 0.4 m/s | 90 / 90 % | 89 / 90 % | | stand: drift | 0.000 m/s | 0.002 m/s |

Drift while turning on the spot is 0.023-0.040 m/s (2026-10-12: 0.016-0.018). None of the 208 robots fell. It
overshoots forward commands a little more than its predecessor, most at 0.3 m/s; it turns closer to the command.

**Climbing** (32 robots held at 0.5 m/s for 12 s):

| | 2026-10-12 | 2026-10-13 |
|---|---|---|
| up-stairs, 9 / 12 / 14 cm risers, 60 cm treads | 32 / 32 / 32 of 32, none fell | 32 / 32 / 32 of 32, none fell |
| deck-and-rail tiles, 12.5 cm decks, 13 cm rails (each +-15 %): onto a deck | 15 of 32, 2 fell | 16 of 32, none fell |
| deck-and-rail tiles, 15.6 cm decks, 16 cm rails | 12 of 32, 1 fell | 16 of 32, none fell |

(On the deck tiles a robot facing a gap walks through it, so "climbed" counts the ones whose heading met a deck.)

## How it was trained

Resumed from the 2026-10-12 policy's checkpoint (iteration 5999) in five segments, each from a checkpoint of the
one before, about 3,300 iterations in all on an RTX 3500 Ada laptop GPU (7-10 s an iteration: the GPU was held at a
48 W power cap). `../../training/README.md`, "The stance fine-tune", has what each segment changed and what it
measured; in short: the centre of mass over the feet, a quarter of the commands stops, feet kept down while
standing (both feet of one end most of all), the stand-still pose term held off for the first second of a stop, a
minimum base height, and the body level with the ground under it. The checkpoint is 9300 of the last segment,
chosen from its six on the slow stops and the turns (the README has the scan).

## Not validated

- **The full sim with ROS.** `run_sim.sh` would not start on this machine on 2026-10-12 (no `cyclonedds` in
  `env_isaaclab`); the measurements above use the sim's environment without it. nav2 has not driven it.
- **Hardware.** Nothing here has run on the robot. It has no actuator delay, as the sim has none.
- **Anything but the square, stairs and the training tiles.** The other lanes are unmeasured.
- **The crate and the railings are not in the height scan**, so the policy does not see them; the route keeps clear
  of them, as nav2 must.
- **A different arm pose.** It trained with the arm folded at its rest. Walking with the arm out is untried; its
  centre of mass would move, and the stance with it.
- **Stops on rough ground or mid-step.** The stance and stop figures are on flat ground. Training stopped robots
  everywhere on its terrain, but that has not been measured.
