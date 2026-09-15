# Training the Rescue Go2 walking policy

The sim's current policy (`logs/rsl_rl/go2_p2dingo/2026-09-12_11-46-13/model_1999.pt`, ported
from P2Dingo) turns precisely and holds a good stance, and it cannot climb the maze's
stairs. This task exists to train one that does both.

It is a copy of P2Dingo's `go2_p2dingo` task, with the parts that produced the turns and
the stance left untouched and four things changed. Each change answers a specific finding
from the 2026 run's training log; the findings come first because they are what justify
the changes.

## What went wrong in the 2026 run

**The terrain curriculum stalled.** `Curriculum/terrain_levels` in that run's TensorBoard
log, out of ten levels:

| iteration | mean terrain level | riser at that level |
|---|---|---|
| 0 | 3.5 | ~11 cm |
| 400 | 0.04 | 5 cm |
| 1000 | 0.75 | ~6 cm |
| 1999 | 1.5 | ~8 cm |

The robot was not failing at that level: 96% of episodes timed out, 4% fell, and the reward
was flat from iteration 500. It was comfortable and not being promoted. The stock rule
promotes only when a robot's straight-line displacement from spawn exceeds 4 m in 20 s, and
P2Dingo's command generator (in-place turns, no heading control, held yaw commands that
trace circles) rarely produces 4 m of displacement however well the robot walks. P2Dingo has
since replaced that rule with one judged on path length (`mdp/curriculums.py`,
`terrain_levels_path`); it is inherited here and is the first-order fix.

**The geometry was not Rescue's.** The maze (`../maze_terrain.py`, `../terrain_cfg.py`) has
10 cm risers on 20 cm treads in walled 1.2 m corridors. Training stairs had 30 cm treads.
`go2_rescue/terrain_cfg.py` explains what replaces them and why, and why the maze's walls
are left out.

**The arm was not the cause, and it is on the training robot anyway.** Tested directly:
with `--arm_mount teleport`, which makes the D1 dynamically inert, the robot failed the
stairs the same way. But the sim runs welded, and a policy should train on the robot it
will drive, so the training robot is the Go2 with the D1 welded to it by the sim's own
`arm_weld.py`: same mount, same 3.152 kg mass model, same servo gains, arm folded at zero.
`go2_rescue/welded_robot.py` has the details and the guard that stops a failed weld from
starting a run.

## What is different from P2Dingo's task

| | P2Dingo `go2_p2dingo` | Rescue `go2_rescue` | why |
|---|---|---|---|
| terrain | stock rough generator | `terrain_cfg.py`: 20 cm and 30 cm stairs, slopes | deployment geometry |
| terrain curriculum | path length (already fixed there) | same | inherited |
| `undesired_contacts` | none | calves and thighs, -1.0 | a shin on a step edge is how a climb stalls |
| `foot_stumble` | none | -0.5, ratio 4 | a toe striking a riser; `mdp/rewards.py` |
| `foot_clearance.target_height` | 0.08 | 0.10 | the riser height |
| robot | stock bare Go2 | Go2 + welded D1, 20 joints, policy scoped to the 12 legs | train on the deployed robot |
| `joint_torques`, `joint_acc`, `joint_vel`, `joint_limits` | Spot's, over `.*` | leg-scoped copies in `mdp/rewards.py` | Spot's ignore joint ids; `.*` is now 20 joints |
| `max_iterations` | 2000 | 4000 | there is now a ladder to climb |
| leg motor | Isaac Lab's ideal DC motor | Unitree's measured torque-speed curve | see below |
| foot friction | fixed 0.8 / 0.6 | randomised 0.3-1.2 | one grip is an assumption |
| pushes | none (the Go2 config disables them) | 1 m/s every 5-10 s | recovery has to be trained |
| joint reset | zero velocity | randomised 1 rad/s | episodes should not all start from rest |
| tilt termination | none | ends past 0.8 rad | a robot on its back is not a sample |
| task ids, experiment dir | `...P2Dingo...`, `go2_p2dingo` | `...Rescue...`, `go2_rescue` | coexists with P2Dingo's install |

Everything else, including the command generator, every posture, gait, ride-height and
limit term and all their weights, is byte-for-byte P2Dingo's as of 2026-09-12. That is
deliberate: those terms are the turns and the stance. The four leg-scoped penalties are
the same maths over twelve columns instead of twenty, at the same weights.

The two new reward weights are first settings, not measurements. Their docstrings say how
they were sized and what to watch.

## What came from MaiRo

Five settings are taken from `mairo-rl-lab-rinam`, a Unitree-derived stack with
demonstrated transfer onto this robot. Its reward set is close to stock and is not where
its transfer comes from; these are. None touches the observation, the action or the
network, so the result stays a drop-in for the sim.

**The motor is the big one.** Isaac Lab's stock Go2 uses an ideal DC motor: a straight line
from 23.5 N·m at rest to zero at 30 rad/s. The real Go2 motor holds full torque to a knee at
13.5 rad/s, is stronger braking than driving, and dies off sharply after. The stock model is
wrong in both directions, and the braking error is the one that matters:

| joint speed | stock drive / brake | real drive / brake |
|---|---|---|
| 8 rad/s | 17.2 / 23.5 | 20.2 / 23.4 |
| 13.5 rad/s | 12.9 / 23.5 | 20.2 / 23.4 |
| 27 rad/s | 2.3 / 23.5 | 3.7 / 4.3 |

Stock braking torque is a flat 23.5 N·m at every speed, because the model clips it against
the effort limit rather than the curve. The real motor gives 4.3 N·m at 27 rad/s. Catching
the body on a step edge is a braking action at high joint speed, so this is exactly the
authority a stair policy learns to rely on and the robot does not have.

`../go2_actuators.py` holds the model, and **both** the sim and this task apply it. That is
deliberate and must stay true: a policy trained on one motor and played back on the other is
running on a robot it never saw. The file sits beside the sim rather than inside this package
for that reason.

Note this changes the robot under the **existing** 2026 policy too. It still loads and still
walks, since the motor is not part of the observation, but it is now playing on a slightly
different robot than it trained on. The comment in `../custom_rl_env.py` says how to revert
for that policy.

**The other four** are domain randomisation, in `go2_rescue/rough_env_cfg.py`. Foot friction
spreads over 0.3 to 1.2 instead of a single pinned value, the body takes a 1 m/s shove every
5 to 10 seconds, joints reset with up to 1 rad/s of velocity instead of at rest, and an
episode ends once the body tilts past 0.8 rad. All four cost training time on purpose: each
removes an assumption the policy could otherwise lean on. Expect the terrain curriculum to
climb more slowly than it would without them.

The PLAY configs pin friction back to the stock fixed values and drop the pushes, so the
probes measure the policy rather than a surface lottery, and stay comparable with the 2026
numbers in `../agent_cfg.py`.

**Not taken: their observation layout.** MaiRo's policy is blind by design, 45 inputs with a
privileged critic, because a real Go2 cannot measure base linear velocity and their
deployment stack has no elevation map to feed a height scan. This task stays perceptive and
235 wide, which is the better stair climber and what the sim already loads. Real-robot
deployment would need that decision revisited first.

## Install

```bash
cd ~/Rescue/Isaac/go2_omniverse/training
./install_task.sh                          # links into $HOME/IsaacLab
ISAACLAB_PATH=/opt/IsaacLab ./install_task.sh
```

Symlinks `go2_rescue/` into Isaac Lab's velocity-config package beside P2Dingo's link. Both
can be installed at once. Verify:

```bash
cd ~/IsaacLab && ./isaaclab.sh -p scripts/environments/list_envs.py | grep Rescue
```

## Train

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Rescue-Unitree-Go2-v0 --headless --num_envs 4096
```

Checkpoints and TensorBoard land in `~/IsaacLab/logs/rsl_rl/go2_rescue/<timestamp>/`. One
GPU trains one of these at a time; P2Dingo's run in progress today (`2026-09-12_16-10-37`)
has to finish first.

At startup the task composes the welded robot to
`Isaac/go2_omniverse/d1_arm/generated/go2_d1_train.usd` (gitignored, rebuilt every run)
and prints a `[weld]` line with the arm mass it applied. If scene creation fails on
`Joint[1-6]` matching no joints, the weld produced a second articulation instead of one;
that is the guard doing its job, and `arm_weld.py` is where to look.

**What to watch, in order.** `Curriculum/terrain_levels` must climb past 4.5, which is the
10 cm riser on the 20 cm stairs, and should reach 7 or more. If it flattens below that with
`Episode_Reward/foot_stumble` or `Episode_Reward/undesired_contacts` also flat, the stairs
are being paid for and not learned, and those two weights are the first lever.
`Curriculum/lin_vel_cmd_levels` should read 1.0 by about iteration 900 at the latest.
`Episode_Termination/base_contact` above ~10% late in the run means the steepest stairs
are costing more than they teach; lower the top of `STAIR_RISER_RANGE` in `terrain_cfg.py`.

## Watch it

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Velocity-Rescue-Unitree-Go2-Play-v0 --num_envs 32
```

The play config draws terrain difficulty at random across a 5x5 grid, so what is on screen
is the full range of stairs. The real test is the maze itself (below).

## Check the turns and the stance survived

P2Dingo's probes work unchanged against the flat play task; they take their height datum
from the environment origin and would read stair relief as gait on generated ground.

```bash
for probe in posture gait swing turn; do
  ./isaaclab.sh -p ~/P2Dingo/Isaac/go2_omniverse/training/measure_$probe.py \
    --task Isaac-Velocity-Rescue-Unitree-Go2-Flat-Play-v0 \
    --checkpoint ~/IsaacLab/logs/rsl_rl/go2_rescue/<run>/model_3999.pt \
    --num_envs 32 --headless --out $probe.txt
done
```

The numbers to hold, from the 2026 policy as recorded in `../agent_cfg.py`: yaw tracking
97.7% at 0.5 rad/s and 95.5% at 1.0, drift while turning 0.076 m/s, resting hip abduction
near -2 degrees, diagonal-pair duty ratio 1.02-1.12 walking. A stair-capable policy that
gives up more than a few points on any of those has traded the wrong thing, and the
place to look first is `foot_clearance.target_height`, the one gait-shaping value that moved.

## Install the result into the sim

1. Copy the run directory (the one model file is enough) to
   `../logs/rsl_rl/go2_rescue/<timestamp>/`.
2. In `../agent_cfg.py`, set `experiment_name` to `go2_rescue`, `load_run` to the timestamp
   and `load_checkpoint` to the model file. Pin them exactly; both are regexes.
3. `git add` the checkpoint. `logs/` is tracked in this repo, so a plain add works.
4. Run the sim on the maze and put it on the stairs. That is the acceptance test.

The three things that would silently break the drop-in are unchanged from P2Dingo's task
and are listed in `../agent_cfg.py`: the observation must stay 235 wide (the height scanner
stays in every variant here, including the flat ones), the network must stay
`[512, 256, 128]`, and the action scale must stay 0.25. Nothing in this task touches any
of them.

## Why the first two runs could not climb

Both runs produced a competent walker that could not ascend a step. Recording the diagnosis
because three plausible causes were tested and refuted, and each would otherwise be retried.

**The measurement that mattered.** `measure_climb.py` holds a forward command on the training
terrain and reports height gained. Run 1, at 4000 iterations: 0 of 64 robots gained 10 cm.
Run 2, at 3300: the same. Every training curve looked healthy throughout both runs. Curves are
not evidence of capability; this probe is.

**The control that located it.** `measure_climb.py` with P2Dingo's checkpoint, on the SAME
task, robot and probe. P2Dingo trained on slopes, rough ground and boxes with NO STAIRS.

| | ours, stairs-heavy | P2Dingo, no stairs |
|---|---|---|
| rose above 10 cm at 1 m/s | 0.0% | 14.1% |
| best height gained | 0.05 m | 1.14 m |

The terrain is climbable, the task is sound and the probe works. Our stairs-dominated terrain
produced a worse climber than a general-purpose policy that never saw a stair.

**Refuted: the curriculum promotion rule.** Real, and fixed (see `terrain_levels_traverse`),
but not sufficient. Run 2 had the fix and still climbed nothing.

**Refuted: friction randomisation.** The obvious suspect, since 0.3 grip on a 20 cm tread
should slide. Swept with the known-good climber at 0.3 / 0.5 / 0.8: it cleared 10 cm on
14.1% / 18.8% / 15.6% of robots. Low grip does not prevent climbing.

**Refuted: the two stair terms added to help.** A calf/thigh contact penalty and a
riser-strike penalty. `measure_climbcost.py` attributes reward to what each robot actually
did: together they cost a climbing robot 0.15 over a 12 s window. They were removed anyway,
being useless and an override of an upstream decision, but they were never the cause.

**The actual finding: climbing does not pay.** From the same attribution, per robot over 12 s:

| | ascending | level ground | difference |
|---|---|---|---|
| gait | 46.1 | 73.3 | −27.2 |
| velocity tracking | 15.1 | 24.8 | −9.6 |
| total of watched terms | 45.6 | 74.8 | −29.3 |

A climbing robot earns 61% of what one on level ground earns, and the gap is the POSITIVE
terms collapsing, not penalties rising. Under a reward that pays for horizontal speed,
climbing is strictly worse and the policy correctly learns to avoid it. No penalty tuning
changes that.

**Why slopes teach stairs and stairs do not.** A stair is a discrete barrier: a policy that
cannot clear a riser gets no gradient toward clearing one, while being paid not to try. A
slope has no barrier, so vertical competence is learned by degrees and transfers to steps.
Hence the rebalanced mix in `go2_rescue/terrain_cfg.py`: stairs from 60% to 30%, slopes from
10% to 30%, rough ground from 10% to 20%, and a riser ceiling of 13 cm rather than 16, since
both runs stalled near 7 cm and range above 13 is range never reached.

**If run 3 also fails to climb**, the next lever is the reward, and the attribution table
above already names it: `gait` accounts for 27.2 of the 29.3 unit gap, 93% of the entire
disincentive to climb. It is the largest positive term in the set at weight 10, it grades
contact TIMING, and stairs necessarily disrupt timing -- so the better the trot, the more a
step costs. Isaac Lab's own Go2 rough config, which does learn stairs, has no gait term at
all; this one inherits it from Spot.

The obvious move is therefore to scale `gait` down as the terrain gets rougher, the way
`GaitRewardScaled` already scales it by commanded speed. Note what this does NOT justify:
simply lowering the weight everywhere, which would give back the trot quality the term exists
to buy and which P2Dingo measured carefully. It has to stay full-strength on flat ground.

A second, weaker lever is the velocity kernel (`base_linear_velocity`'s `std`, 0.3), which
goes flat once the error is large, so on terrain the policy cannot yet track it stops trying;
P2Dingo records 0.5 as discriminating better in exactly that regime, at the risk of the
diagonal-pair limp returning unless `air_time_variance` rises with it.

Neither was changed alongside the terrain, to keep one variable at a time. The evidence that
terrain alone may be enough is that P2Dingo's policy carries this same gait term at weight 10
and climbs anyway -- the only thing that differs is the ground it learned on.

## The wall problem, and why the sim filters the height scan

Found by driving the trained policy in the maze: it refused forward commands and reversed
instead. Reproduced on flat ground by `measure_scan.py`, which overwrites only the height
scan and changes nothing else. Commanded +1.0 m/s forward:

| scan condition | forward delivered |
|---|---|
| clean baseline | +0.53 m/s |
| wall 0.5 m ahead | −0.47 m/s |
| side walls only, a 1.2 m corridor | −0.03 m/s |

The height scan reads more negative for higher ground and clips at −1.0. A 1 m wall reads
−1.165 and clips; the tallest riser in training was 0.16 m, reading about −0.33. The policy's
only learned meaning for a large negative ahead is "a step up", so a wall is a step it cannot
climb and it backs away. In a 1.2 m corridor with a 1.0 m wide scan, a wall is in view
whenever the robot is more than 10 cm off centre, which is most of the time.

**Clamping the scan does not fix it**, which is worth recording because it is the obvious
idea. A clamp only makes the wall a shorter step. Swept at −1.00, −0.85, −0.70, −0.55 and
−0.40, every value still reversed the robot, and −0.70 was the *worst* at −1.19 m/s, where the
wall reads most like a steep but plausible stair. A clamp of v implies a step of
(0.335 − 0.5 − v) m, so even −0.40 says 0.24 m, above anything the policy has climbed. The
value that would read as climbable is about −0.33, and the maze's own risers read −0.57, so
that clamp erases the steps the robot is there to climb.

So the sim REMOVES walls from the scan instead: `height_scan_walls_removed` in
`../custom_rl_env.py` replaces rays past −0.75 with the median of the walkable ones. The
threshold has room — the maze's worst case is a 4-step flight of 10 cm risers reaching −0.57.
The policy then sees the floor continuing and nav2 keeps sole ownership of walls, which is
where that decision belongs.

**Training with walls was tried and did NOT work**, so the filter is the fix rather than a
workaround. Run 3 carries 5% tall-obstacle tiles at the maze's 1 m height. Measured on its
checkpoint at iteration 3850, against the policy that had no wall exposure at all:

| scan condition | no wall exposure | 5% wall tiles |
|---|---|---|
| clean baseline | +0.53 m/s | +1.01 m/s |
| wall ahead | −0.47 | −0.50 |
| corridor, side walls only | −0.03 | −0.28 |

Unchanged where it matters and worse in a corridor. The reason is a mismatch between what was
trained and what the maze presents: scattered 0.4 m boxes make ISOLATED clipped patches that
a robot walks around, while a 1.2 m corridor makes a CONTINUOUS clipped band down both sides.
Five percent of tiles of the wrong pattern teaches nothing.

Two consequences. The filter stays, and it is sufficient -- with walls removed from the scan
the policy sees its clean baseline and tracks forward at 1.01 m/s. And the wall tiles should
come OUT of the terrain mix next run: with the filter active the policy never meets a wall at
inference, so those tiles are redundant and cost 5% of the ladder that slopes and rough ground
use better. Left in for run 3 only because restarting a working run to reclaim 5% is not worth
it.

If wall exposure is ever attempted again, it needs CORRIDOR geometry -- parallel walls the
robot walks between -- not scattered obstacles.

## The seeded benchmark, and what it overturned (2026-09-14)

`measure_climb.py` cannot rank checkpoints. Run twice on model_7700 it reported 12.5 % and
23.4 % of robots clearing 10 cm: it plays the rough PLAY task, whose tiles draw a random type
and difficulty from an unseeded generator, so every run is different ground with a handful of
robots on any one stair. Its numbers above, including the P2Dingo control, are pooled over
slopes, boxes and stairs alike and do not say which of those the robots climbed.

`measure_bench.py` replaces it for anything that compares checkpoints. It puts every robot on
the maze's own geometry (10 cm risers, 20 cm treads) -- flat, a 4-step flight up and down, an
11-step flight up and down -- facing the flight from 1.2 m, seeded, with friction pinned, and
on the robot the SIM drives: no base-mass or CoM randomisation. Two seeds agree to within a
point on every metric. `bench_table.py` lays JSON outputs side by side.

**The robot is the finding.** Every earlier probe ran the PLAY task with training's mass and
CoM randomisation still on. The CoM range is 0..+5 cm FORWARD (rough_env_cfg.py, for the real
robot's Livox and camera head), so the sim's nominal robot sits at the very edge of it, and the
policy's forward speed follows that offset almost linearly. model_7650, flat, 0.5 m/s:

| base CoM x | tracking | onto the first step, 4-step flight up |
|---|---|---|
| 0 (the sim) | 80 % | 0 % |
| +2.5 cm | 103 % | 35-53 % |
| +5 cm | 116 % | 100 % |

Added mass barely matters (+1 kg: 84 %, +3 kg: 87 %). The earlier 96.5 % forward tracking from
`measure_axis.py` was measured on the randomised robot, which is why it never matched the
"unresponsive forward" seen in the sim.

**Nothing trained so far climbs the maze's stairs** on that robot, P2Dingo included:

| | 7650 | 7700 (installed) | P2Dingo |
|---|---|---|---|
| 0.5 m/s: onto the first step up | 0 % | 0 % | 0 % |
| 1.0 m/s: onto the first step up | 100 % | 99 % | 99 % |
| 1.0 m/s: whole 4-step flight up | 8 % | 0 % | 0 % |
| 1.0 m/s: fell on the flight up | 0 % | 0 % | 15 % |
| 0.5 m/s: whole 11-step flight down | 97 % | 100 % | 31 % |

At 1.0 m/s the robots mount the first step, stall, back away and turn off the flight --
body-frame speed returns to forward while the base keeps moving away from the stairs. And run
3's terrain curriculum sat at level ~1.1 of 10 for its entire life (`Curriculum/terrain_levels`
in all three segments), which is a ~6 cm riser: the maze's 10 cm is level ~6. The policy was
never trained at the height it is asked to climb.

**The feet gather under the body, and that is Rescue's, not the robot's.** Touchdown relative
to each foot's own thigh joint, yaw frame, flat ground, 0.5 m/s:

| | 7650 | P2Dingo |
|---|---|---|
| front foot at touchdown | -10.4 cm (behind the hip) | +5.3 cm |
| rear foot at touchdown | +17.8 cm (ahead of the hip) | +4.7 cm |
| front, against Raibert's neutral point v*T_stance/2 | -15.4 cm | -1.0 cm |
| rear, against the neutral point | +12.8 cm | -1.2 cm |
| same-side front-to-rear spacing at front touchdown | 19.7 cm | 49.6 cm |

Hips are 38.7 cm apart, so the Rescue policies land front and rear feet half a hip spacing
apart. P2Dingo's policy lands on the neutral point -- and still does not climb, so better
placement alone is not the stair fix.

**The value-loss explosions are a runaway in the policy's own output.** Actions are unclipped,
the last action is an observation, and `action_smoothness` is ||a_t - a_{t-1}||. One large
output feeds its own next input; joint targets saturate at the torque limits, so physics and
every other reward term stay bounded while `action_smoothness` alone reaches ~1e7 per step --
exactly the signature in run 3's logs -- and the value loss overflows into the NaN that ends
the run ("normal expects all elements of std >= 0.0"). It is not a training artefact:
deterministic rollouts of model_7650 in `measure_bench.py` reached |a| = 28 682 on one robot,
where 99.99 % of normal output is below 8.5. A fine-tune from 7650 with nothing changed blew up
within 155 iterations. The experiments clip actions at +-20 (`Go2RescueExpPPORunnerCfg`), which
never touches normal output. The sim's `RslRlVecEnvWrapper(env)` in `omniverse_sim.py` does not
clip either, so the same runaway can happen there.

The short fine-tune experiments that follow from these three findings are in
`go2_rescue/experiments.py`.

## The fine-tune experiments (2026-09-14)

Seven arms of `go2_rescue/experiments.py`, each 1000 iterations resumed from model_7650 with
actions clipped at +-20, benchmarked on the sim's robot with `measure_bench.py` (seed 1). The
JSON and text outputs are in `bench_results/2026-09-14/`, and
`python bench_table.py a=arms/Com_v1.0.json b=arms/Thigh_v1.0.json` reproduces any column.
None of the seven blew up (worst value loss 13; the unclipped control reached 1.4e16 and
crashed).

| | base | control | com | com, 2nd seed | riser | stairfwd | thigh | placement -30 |
|---|---|---|---|---|---|---|---|---|
| flat tracking, 0.5 m/s | 80 % | 92 % | 108 % | 107 % | 104 % | 110 % | 101 % | 113 % |
| turn tracking, 1 rad/s | 91 % | 99 % | 101 % | 105 % | 104 % | 101 % | 106 % | 99 % |
| front foot vs hip at touchdown, 1.0 m/s | -4.5 | -4.8 | -5.0 | -3.3 | -6.5 | -3.3 | -6.5 | **+3.2** cm |
| same-side front-rear spacing, 1.0 m/s | 31 | 31 | 30 | 34 | 29 | 35 | 27 | **47** cm |
| pitch wobble, 1.0 m/s | 0.53 | 0.52 | 0.37 | 0.47 | 0.58 | 0.56 | 0.73 | 1.07 deg |
| onto the first step up, 0.5 m/s | 0 % | 0 % | 92 % | 94 % | 0 % | 0 % | 100 % | 51 % |
| onto the first step up, 1.0 m/s | 100 % | 100 % | 100 % | 100 % | **0 %** | **0 %** | 100 % | 100 % |
| 4-step flight up, 1.0 m/s | 8 % | 0 % | 35 % | **89 %** | 0 % | 0 % | 55 % | 12 % |
| 11-step flight up, either speed | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % |
| 4-step flight down, 0.5 m/s | 100 % | 6 % | 100 % | 100 % | 100 % | 17 % | 100 % | 100 % |

(The first placement arm, weight -1.0, is not in the table: the term logs a per-step mean and
fires on ~0.16 of steps, so it charged -0.02 instead of the intended -0.8 and changed nothing.
`ExpPlacementW30EnvCfg` is the corrected one.)

**The second seed of `com` is the most important column.** Its flat-ground numbers repeat the
first seed's to within a few percent and a couple of centimetres, so flat metrics survive a
change of training seed. The flight completion at 1.0 m/s does not: 35 % and 89 % from the
same settings. Stair outcomes of a 1000-iteration fine-tune are dominated by training noise, and
no single-seed stair difference in this table can be read as an effect.

What does survive that:

- **The CoM range is a fix.** Every arm that carries it tracks at 101-113 %; the two that do not
  track at 80-92 %. It is also the only change common to every arm that walks onto a step at
  0.5 m/s.
- **The riser range change makes the policy refuse stairs.** Both arms that carry it (riser and
  stairfwd) mount nothing at 1.0 m/s, where every other checkpoint, the base included, mounts
  the first step 100 % of the time. Practising at maze height before the policy can climb teaches
  avoidance, which is the "stairs are a barrier" finding above, again.
- **The placement penalty moves the feet, at a price.** At -30 it is the only arm whose feet moved:
  the rear foot lands 2.5 cm from Raibert's neutral point (from ~10) and the front-rear spacing
  opens from ~30 to 47 cm, close to P2Dingo's. The penalty was still falling at the end, so the
  front feet (9.5 cm short) were still moving. Pitch wobble doubled and turn drift rose from
  0.05 to 0.09 m/s. Its stair numbers are inside the noise.
- **Relaxing the thigh posture term does not move the feet.** Front touchdown -6.5 cm, spacing 27.
- **Nothing climbs an 11-step flight, and ascent is still unsolved.** The untested levers are the
  ones this README already names: a terrain curriculum that actually advances on stair tiles
  (run 3 sat at level ~1.1) and scaling `gait` down on rough ground (93 % of the measured
  disincentive to climb).

## Flat ground, and why the first flat policy is not installed (2026-09-14)

**The maze is flat now.** No policy climbs its stairs on the sim's robot, so the maze is built on
one floor: `elevation_chance=0.0` in `../terrain_cfg.py` (the chance a maze cell changes height,
which is where every staircase came from; 0.15 brings them back). Checked over 20 generated
mazes: the only surfaces left are the floor slab and the 1 m walls.

**The sim clips actions at +-20**, from `clip_actions` in `../agent_cfg.py`, so it matches what
Rescue's policies now train with. The training task carries both fixes that survived the
experiments: the symmetric CoM range and the action clip.

**The first flat policy trained with the arm** (`go2_rescue_flat/2026-09-14_21-06-31_flat_arm`,
3000 iterations from scratch) is worse than P2Dingo's flat policy on the same benchmark and the
same welded, measured-motor robot:

| flat, measure_bench.py --flat_only | P2Dingo flat (15-06-39) | Rescue flat, 2000 it | Rescue flat, 3000 it |
|---|---|---|---|
| forward tracking, 0.5 / 1.0 m/s | 98 / 100 % | 100 / 107 % | 111 / 103 % |
| turn tracking at 1.0 rad/s | 104 % | 76 % | 83 % |
| drift while turning | 0.05 m/s | 0.06 | 0.13 |
| front / rear foot vs its hip at touchdown, 0.5 m/s | +7.1 / +5.5 cm | -6.7 / +23.6 | -6.9 / +16.9 |
| same-side front-rear spacing, 0.5 m/s | 52 cm | 18 | 27 |
| backward within 1 s of a 1.0 m/s forward request | 0 % | 57 % | 18.5 % |

Two conclusions. The gathered feet are not caused by the stairs: they appear on a plane. And
they are not caused by the arm being ON the robot either, because P2Dingo's policy, which never
trained with it, walks this welded robot with its feet on the neutral point and turns at 104 %.
What degrades the gait is something Rescue's task added to P2Dingo's. On flat ground that is:
training WITH the arm, the measured motor curve, MaiRo's randomisation (friction 0.3-1.2, 1 m/s
shoves, joint-velocity reset, tip-over termination), and the 10 cm clearance target (P2Dingo's
is 8). `go2_rescue/experiments.py` has one flat ablation per factor, plus `FlatP2D` with all of
them removed, which must reproduce P2Dingo or the list is incomplete.

**The ablation (2026-09-15) says it is the arm.** Five 2000-iteration flat runs, each removing one
factor from the Rescue flat task, plus `FlatP2D` with all of them removed and a second seed of
the unchanged task. All benchmarked on the sim's welded, measured-motor robot
(`bench_results/2026-09-15/flat_ablation/`, turn rates in `turn_rates/`):

| | P2Dingo flat | all removed | Rescue, seed 42 | Rescue, seed 2 | no MaiRo DR | **no arm** | stock motor | 8 cm clearance |
|---|---|---|---|---|---|---|---|---|
| front / rear foot vs neutral point, 0.5 m/s | -1 / -2 cm | -2 / -1 | -12 / +18 | -9 / +3 | -7 / +14 | **-4 / -3** | -10 / +17 | -10 / +17 |
| same-side spacing, 1.0 m/s | 66 cm | 58 | 32 | 36 | 32 | **54** | 35 | 32 |
| stride / stance, 0.5 m/s | 28 cm / 346 ms | 21 / 253 | 25 / 234 | 12 / 131 | 28 / 274 | 17 / 189 | 23 / 223 | 27 / 250 |
| forward tracking, 0.5 / 1.0 m/s | 98 / 100 % | 95 / 97 | 100 / 107 | 104 / 102 | 101 / 104 | 106 / 102 | 98 / 107 | 101 / 105 |
| turn tracking, 0.2 / 0.5 / 1.0 rad/s | 7 / 98 / 104 % | 58 / 99 / 99 | - / - / 76 | 75 / 106 / 105 | - / - / 89 | 51 / 124 / 109 | - / - / 99 | - / - / 102 |
| drift turning at 1.0 rad/s | 0.05 m/s | 0.06 | 0.06 | 0.03 | 0.07 | 0.03 | 0.05 | 0.05 |
| backward after a 1.0 m/s request | 0 % | 0 | 57 | 0 | 0 | 0 | **100** | 0 |
| pitch wobble, 1.0 m/s | 0.73 deg | 0.70 | 1.17 | 0.17 | 0.93 | 0.46 | 0.95 | 0.94 |

(The installed stair policy, model_7700, on the same robot: turn 65 / 96 / 99 %, drift 0.06.)

`FlatP2D` reproduces P2Dingo, so the list of differences is complete. Of the four single
removals, only taking the arm off the TRAINING robot brings the feet back to the neutral point;
the three that keep the arm all gather the rear feet 20-23 cm ahead of the hips, and the second
seed of the unchanged task avoids that only by switching to a 131 ms, 12 cm shuffle. Both runs
without the arm walk well; none of the five with it does. Every run is one seed, but that split
is 2 of 2 against 0 of 5.

The policy that never trained with the arm walks the welded robot better than every policy that
did, which is the opposite of why the arm was added to training. Why is not established: the arm
raises the centre of mass and pitch inertia, and the policy may be finding that feet under the
body are the cheapest way to control pitch during learning, while a policy that learned without
that pressure copes with the arm fine once it is there.

None of the flat candidates tracks a 0.2 rad/s turn well (P2Dingo's flat policy barely turns at
all at that rate). nav2's DWB never commands one: its `min_speed_theta` is 0.8 rad/s.

**Installed 2026-09-15: the "all removed" run** (`go2_rescue_flat/2026-09-15_08-09-04_abl_FlatP2D`,
model_1999), copied into `../logs/rsl_rl/` and selected in `../agent_cfg.py`, whose header has
its numbers and the rollbacks. Chosen over "no arm" for tracking accuracy (turns 99 % at 0.5 and
1.0 rad/s against 124 / 109 %).

## Not done here

- **No actuator delay.** The motor model supports one (`min_delay`/`max_delay`, in 5 ms
  physics steps) and it is set to zero, matching MaiRo and matching this sim, which applies
  the policy with no delay. A real 50 Hz controller never acts on the state it just read, so
  a deployment-bound policy wants 1 to 4 here. Raise it in the same edit that adds delay to
  deployment, not before.

- **The maze itself is not the training terrain.** Its generator (`../maze_terrain.py`) is
  unseeded, builds dead ends a velocity command cannot avoid, and its walls would pin a
  non-navigating robot. Walls are deliberately absent from training: keeping away from
  them is the nav stack's job, and a gait policy with its own opinion about walls would
  argue with it. The stairs are matched by geometry. If the policy climbs the training
  stairs and not the maze's, the difference to look at is the corridor: a 1.2 m corridor
  puts a wall inside the scan's 1.0 m width whenever the robot is more than 7.5 cm off
  centre, an input the policy has never seen.
- **The per-foot ground datum is in place but unvalidated** by a full run, here or in
  P2Dingo. Its docstring in `mdp/rewards.py` has the reasoning.
- **The arm trains folded.** Its joints are PD-held at zero and never randomised, which is
  the pose it rides in while the dog walks. A policy robust to the arm being raised or
  extended mid-walk would need the arm's position targets randomised at reset; that is
  extra machinery and not the deployment case, so it is left out.
