# Go2-W + D1 walking policy for the K-Rail Square

The wheeled Go2's policy for the K-Rail Square: steps onto and off the 12.2 cm deck, over the K-rails round the crate,
and rolls where rolling will do. Trained in this repository on the sim's own Go2-W with the D1 folded on its back
(`../../training/go2_rescue/krail_go2w_env_cfg.py`, `Isaac-Velocity-Rescue-KRail-Go2W-v0`), with the Go2 policy's
final task: its rewards and stance terms, terrain and ladder, mirror training. Exported on 2026-11-01 by
`../../training/export_krail.py`. The rescue sim's default policy from that date, for its default robot.

```bash
cd ~/Rescue/Isaac/go2_omniverse
./run_sim.sh --level square          # the Go2-W and this policy; --robot go2 for the legged Go2 and its own
```

```
policy.pt        TorchScript: (1, 749) observation -> (1, 16) raw action, the policy mean, clamped at +-20
policy.onnx      the same network; input `obs`, output `actions`
params/deploy.yaml   the deploy contract: the Go2 bundle's, with the wheels' action and drive added
params/env.yaml      the training run's environment, which the sim reads the height scanner from
params/agent.yaml    the training run's runner config
params/export.json   source run and checkpoint, sha256 of each file, TorchScript/ONNX agreement
eval/                the measurements below, as JSON
```

The source checkpoint, for fine-tuning, is `../../logs/rsl_rl/go2w_krail/2026-10-31_18-06-25_w1/model_3000.pt`.

## What it expects

The sim checks the robot, legs, wheels, gains, control rate, action layout and observation width against the
contract before the policy gets the robot, and stops if anything differs. It refuses this policy on the Go2.

| | |
|---|---|
| Robot | The sim's Go2-W (`../../go2w.py`) with the D1 welded at the Go2's mount, 22.7 kg |
| Observation | 749: `base_lin_vel`, `base_ang_vel`, `projected_gravity`, `velocity_commands`, the 12 legs' `joint_pos_rel`, `joint_vel_rel` of the 12 legs and then the 4 wheels, `last_action` (16), then the height scan. Unscaled and unclipped, except the scan at +-1 |
| Height scan | 693 rays, a 1.6 x 1.0 m grid at 5 cm centred on the base, aligned with its yaw, `base_z - hit_z - 0.5`. Every cell, no camera mask |
| Commands | vx +-1.0 m/s, vy +-0.5 m/s, yaw +-1.5 rad/s; the sim clamps to these. Sideways is narrower than the Go2's 0.8: a wheel does not roll sideways |
| Action | 12 leg position targets, `default + 0.25 * action`, Isaac Lab leg order (hips FL FR RL RR, thighs, calves); then 4 wheel velocity targets, `5 * action` rad/s, FL FR RL RR. Clamped at +-20 inside `policy.pt`, as training clamped it |
| Legs | Unitree's Go2 HV motor curve (`go2_actuators.py`), kp 25, kd 0.5, 50 Hz |
| Wheels | Velocity drives, kd 0.5 N*m*s/rad, 23.5 N*m, 30 rad/s, as Unitree's own Isaac Lab model of the Go2-W |

Like the Go2's, it reads `base_lin_vel`, which a real robot would need a velocity estimate for. It has not been run on
hardware.

## Measured (2026-11-01), against the Go2's 2026-10-13 policy

All on GPU physics, as the Go2's were. The Go2's figures are its README's.

**K-Rail Square** (`../../training/measure_square.py`: the sim's own environment and hall, without the lidar, the
camera, ROS and the arm's DDS firmware; the course drawing's route, START pad, up the 12.2 cm step, over two K-rails
round the crate, down the step, END pad, driven by a nav2-like driver that turns on the spot and never strafes):

| | Go2, 2026-10-13 | Go2-W, 2026-11-01 |
|---|---|---|
| 0.5 m/s, 10 cm rails (the square as built) | 20 of 20, 19.1 s | **20 of 20**, 20.2 s, no falls |
| 0.3 m/s | 10 of 10, 27.6 s | 10 of 10, 32.7 s |
| 0.8 m/s | 10 of 10, 15.0 s | 10 of 10, 14.6 s |
| 0.5 m/s, 15 cm rails (`--k-rail-height 0.15`) | 10 of 10 | 10 of 10, 20.2 s |
| 0.5 m/s, 20 cm rails | 10 of 10 | 10 of 10, 20.4 s |
| along the 0.5 m/s route: speed / turn rate held, where the command was steady | 106 / 98 % | 99.7 / 71 % |
| ... mean absolute error, vx / vy / wz | 0.033 / 0.021 / 0.051 | 0.017 / 0.013 / 0.100 |

The turn rate is one turn's: the route's first, 83 degrees to the right at the square's west entrance, straight out of
the 1.7 m walk north. The driver cuts 0.5 m/s to a 1 rad/s turn on the spot there; the robot turns at the full rate
for about 0.6 s, then at 0.15-0.4 rad/s for about a second, then picks up. Over that turn it holds 46-62 % of the
rate (two logged trials); every other turn on the route holds 102-110 %. Started from a standstill at the same spot it
turns at 100-102 % either way, and on the START pad, the hall floor and the deck at 90-106 %. On CPU physics the
route's turns held 102 %. It costs about a second a route.

**Held commands on flat ground** (`measure_tracking.py`, `eval/tracking_flat.json`; 16 robots each, 12 s, the first 3
dropped; velocities in the yaw frame):

| command | Go2 | Go2-W | | command | Go2 | Go2-W |
|---|---|---|---|---|---|---|
| forward 0.3 m/s | 110 % | 99 % | | turn on the spot 0.5 rad/s | 102 % | 102 % |
| forward 0.5 m/s | 106 % | 104 % | | turn on the spot 1.0 rad/s | 99 % | 104 % |
| forward 0.8 m/s | 104 % | 106 % | | turn on the spot 1.5 rad/s | 99 % | 102 % |
| forward 1.0 m/s | 102 % | 106 % | | turn on the spot -1.0 rad/s | 99 % | 105 % |
| backward 0.5 m/s | 87 % | 103 % | | arc, 0.5 m/s and 0.5 rad/s | 106 % | 109 % |
| left / right 0.4 m/s | 89 / 90 % | 90 / 90 % | | stand: drift | 0.002 m/s | 0.004 m/s |

Drift while turning on the spot is 0.014-0.029 m/s (the Go2's 0.023-0.040). None of the 208 robots fell.

**Stance and stopping** (`../../training/measure_stance.py`, `eval/stance.json`; 16 robots a case on flat ground,
the nominal robot, the command held 4 s and then set to zero, as letting go of the sim's drive keys does). Its feet
here are the wheels' axles:

| | Go2 | Go2-W |
|---|---|---|
| stopped from 0.3 / 0.5 / 0.8 / 1.0 m/s forward: robots with both rear feet up over 0.1 s | 2 / 1 / 0 / 0 of 16 | 0 / 0 / 0 / 0 of 16 |
| ... highest a rear wheel rose | 8-11 cm (a step) | 0.1-0.7 cm |
| ... nose down at most, mean (worst robot) | 3-5 deg (9 deg) | 1.9-4.7 deg (4.7 deg) |
| ... centre of mass behind the front feet, least of any robot | 2.6-10 cm | 12.7-13.0 cm |
| ... how far it went after the command went to zero | | 1 / 4 / 14 / 21 cm |
| stopped from a 0.5 m/s, 0.5 rad/s arc: robots with both rear feet up | 0 of 16 | 0 of 16 |
| front feet's share of the weight, walking forward 0.3 / 0.5 / 0.8 / 1.0 m/s | 62 / 54 / 48 / 45 % | 50 / 50 / 49 / 48 % |
| front feet's share standing, 2 s after a stop | 51-56 % | 45-48 % |
| all four feet down, 2 s after a stop | 82-100 % | 100 % after every stop |
| feet's distance from the centre line, standing after a stop | 15-21 cm | 14-16 cm after forward stops; 10-11 cm after sideways, backward and turning ones |
| base height over its feet, standing | 0.29-0.31 m | 0.26-0.28 m (0.32 in the default pose) |

No sideways, backward or turning stop lifted both rear wheels either (the most a rear wheel rose was 4.3 cm, stopping
from sideways). It stands 4-6 cm lower than its default pose.

**Climbing** (32 robots held at 0.5 m/s for 12 s):

| | Go2 | Go2-W |
|---|---|---|
| up-stairs, 9 / 12 / 14 cm risers, 60 cm treads | 32 / 32 / 32 of 32, none fell | 32 / 32 / 32 of 32, none fell |
| deck-and-rail tiles, 12.5 cm decks, 13 cm rails (each +-15 %): onto a deck | 16 of 32, none fell | 16 of 32, none fell |
| deck-and-rail tiles, 15.6 cm decks, 16 cm rails | 16 of 32, none fell | 14 of 32, none fell |

(On the deck tiles a robot facing a gap walks through it, so "climbed" counts the ones whose heading met a deck.)

## How it was trained

From scratch, one run of 3000 iterations at 4096 robots, 6 h 50 min on an RTX 3500 Ada laptop GPU (8.1 s an
iteration). `../../training/README.md`, "The Go2-W's K-Rail Square policy", has what differs from the Go2's task and
why. The run was stopped at 3000: from 2500 on, its tracking errors, reward and terrain level had stopped moving.
The checkpoint is 3000, chosen over 2000, 2500, 2800 and 2900 on the measurements above. 2800 and 2900 made the
route's first turn at 73-77 % of the rate, but they overshoot every turn on flat ground by 8-25 %, drift 0.010-0.013
m/s standing, and stop from a sideways walk with their wheels 5 cm from the centre line.

## Not validated

- **The full sim with ROS and nav2.** `run_sim.sh` with this policy passes its smoke run (every level loads, it lies
  down in the cup demo, stands up and walks 2.25 m in 4.7 s on the gravel lane at 0.5 m/s). nav2 has not driven it,
  and VIP-Rescue's K-Rail Square tree has not been run with it.
- **Hardware.** Nothing here has run on a Go2-W. It has no actuator delay, as the sim has none. The legs are driven by
  the Go2's measured motor curve; the Go2-W's URDF gives its calves 35.55 N*m to the Go2's 23.4, so the real knee is
  likely stronger than the one it trained with. The wheel drive is Unitree's Isaac Lab model's, not measured.
- **The D1 on a Go2-W.** It trained with the arm on the Go2's mount, folded at rest. Whether the team's Go2-W carries
  it there is an assumption.
- **The turn straight out of a walk**, above. nav2's controllers ramp their commands, which may avoid it, or not.
- **Anything but the square, stairs and the training tiles.** The other lanes are unmeasured.
- **The crate and the railings are not in the height scan**, so the policy does not see them; the route keeps clear
  of them, as nav2 must.
- **Stops on rough ground or mid-step.** The stance and stop figures are on flat ground.
