# Go2 + D1 walking policy for the K-Rail Square

A walking policy for the rescue sim's robot (the Go2 with the D1 welded on its dock rails, folded at its rest)
that steps up onto a deck, over K-rails, and down off it, and tracks the velocity command. Trained in this
repository (`../../training/go2_rescue/krail_env_cfg.py`, task `Isaac-Velocity-Rescue-KRail-Go2-v0`) and exported
on 2026-10-12 by `../../training/export_krail.py`.

```bash
cd ~/Rescue/Isaac/go2_omniverse
./run_sim.sh --level square --policy policies/go2_krail_2026-10-12
```

```
policy.pt        TorchScript: (1, 741) observation -> (1, 12) raw action, the policy mean, clamped at +-20
policy.onnx      the same network; input `obs`, output `actions`
params/deploy.yaml   the deploy contract (`rescue_sim/deploy_contract.py`)
params/env.yaml      the training run's environment, which the sim reads the height scanner from
params/agent.yaml    the training run's runner config (of the last segment, see below)
params/export.json   source run and checkpoint, sha256 of each file, TorchScript/ONNX agreement
eval/                the measurements below, as JSON
```

The source checkpoint, for fine-tuning, is `../../logs/rsl_rl/go2_krail/2026-10-11_23-46-04_v2y/model_5999.pt`.

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

## Measured (2026-10-12)

In the rescue sim's own environment (`../../training/measure_square.py`: `make_env_cfg`, the competition hall,
the sim's checks on the bundle; without the lidar, the camera, ROS and the arm's DDS firmware), GPU physics. The
route is the course drawing's, driven by a nav2-like driver (turns on the spot when off heading, never strafes):
START pad, along the outside of the west edge, up the 12.2 cm step, over two K-rails round the crate, down the
12.2 cm step, to the END pad.

| K-Rail Square | routes completed | fell | mean time |
|---|---|---|---|
| 0.5 m/s, 10 cm rails (the square as built) | **20 of 20** | 0 | 19.1 s |
| 0.3 m/s | 10 of 10 | 0 | 27.9 s |
| 0.8 m/s | 10 of 10 | 0 | 14.9 s |
| 0.5 m/s, 15 cm rails (`--k-rail-height 0.15`) | 10 of 10 | 0 | 19.4 s |
| 0.5 m/s, 20 cm rails (`--k-rail-height 0.20`) | 8 of 10 | 0 | 20.2 s |

At 20 cm the two others stopped short of a rail and did not fall. Along the 0.5 m/s routes the base held 101 % of
the commanded speed and 98 % of the commanded turn rate where the command was steady, with mean absolute errors of
0.024 m/s forward, 0.011 m/s sideways and 0.056 rad/s in yaw.

For comparison, on the same test the 2026-09-18 bundles (`../go2_policies_2026-09-18/rough` and `/flat`) reached
the deck in none of 10 trials each.

Held commands on flat ground (`measure_tracking.py`, 16 robots each, 12 s, the first 3 dropped; the nominal robot,
friction 0.8 / 0.6; velocities in the yaw frame):

| command | achieved | | command | achieved |
|---|---|---|---|---|
| forward 0.3 m/s | 105 % | | turn on the spot 0.5 rad/s | 107 % |
| forward 0.5 m/s | 102 % | | turn on the spot 1.0 rad/s | 104 % |
| forward 0.8 m/s | 100 % | | turn on the spot 1.5 rad/s | 102 % |
| forward 1.0 m/s | 100 % | | turn on the spot -1.0 rad/s | 106 % |
| backward 0.5 m/s | 92 % | | arc, 0.5 m/s and 0.5 rad/s | 103 % |
| left / right 0.4 m/s | 90 % / 90 % | | stand | 0.000 m/s |

Drift while turning on the spot is 0.017 m/s; the yaw rate wanders 0.04-0.05 rad/s while walking straight. None of
the 208 robots fell.

Climbing (32 robots held at 0.5 m/s for 12 s):

| | climbed | fell | speed held |
|---|---|---|---|
| up-stairs, 9 cm risers, 60 cm treads | 32 of 32 | 0 | 102 % |
| up-stairs, 12 cm risers | 32 of 32 | 0 | 101 % |
| up-stairs, 14 cm risers | 32 of 32 | 0 | 102 % |
| deck-and-rail tiles, 12.5 cm decks, 13 cm rails (each +-15 %) | 15 of 32 onto a deck | 2 | 101 % |
| deck-and-rail tiles, 15.6 cm decks, 16 cm rails | 12 of 32 onto a deck | 1 | 91 % |

(On the deck tiles a robot facing a gap walks through it, so "climbed" counts the ones whose heading met a deck.)

## How it was trained

About 12 hours of 4096 robots on an RTX 3500 Ada laptop GPU (6.5 s an iteration), in four segments, each resumed from the last one's
checkpoint (`../../training/README.md`, "The runs", has the measurements behind each change):

1. iterations 0-2000: stock rough-Go2 rewards in the yaw frame, a sharp tracking pair, no penalty on attempting a
   step, decks, K-rails and stairs;
2. 2000-2400: every minibatch also mirrored left-right (turning had been lopsided);
3. 2400-3000: plus a mirror loss (the mirrored batches alone left the mean lopsided and inflated the noise);
4. 3000-5999: yaw tracking weighted up (1.5 and 1.0 from 1.0 and 0.5).

## Not validated

- **The full sim with ROS.** `run_sim.sh` would not start on this machine on 2026-10-12: the `env_isaaclab`
  environment has no `cyclonedds`, which the simulated arm's DDS link imports. The measurements above use the sim's
  environment without it. nav2 has not driven it.
- **Hardware.** Nothing here has run on the robot. It has no actuator delay, as the sim has none.
- **Anything but the square, stairs and the training tiles.** The other lanes (gravel, stepfields, ramps,
  pallets, the 45-degree stairs, the maze) are unmeasured.
- **The crate and the railings are not in the height scan** (the hall's scan mesh leaves them out), so the
  policy does not see them; the route keeps clear of them, as nav2 must.
- **A different arm pose.** It trained with the arm folded at its rest. Walking with the arm out is untried.
