# Go2 policies — flat + rough terrain

Two trained Unitree Go2 locomotion policies exported from the `Go2_RL_workflow` Isaac Lab
project on 2026-09-18, for import into another simulator.

```
flat/    policy.onnx  policy.pt  params/{deploy,env,agent}.yaml
rough/   policy.onnx  policy.pt  params/{deploy,env,agent}.yaml
deploy_contract.py    dependency-free reader for params/deploy.yaml (yaml + stdlib)
```

`params/deploy.yaml` is **machine-generated from the live training environment**, not
hand-written, so it cannot drift from what the policy actually trained against. Read it
rather than transcribing the numbers below — `deploy_contract.py` will do that for you and
refuses a contract it cannot honour instead of guessing.

---

## Which policy is which

| | flat | rough |
|---|---|---|
| Source run | `unitree_go2_flat_dr_omni` 2026-09-15_14-09-12 | `unitree_go2_stepfield_spec_unitree` 2026-09-16_17-09-48 |
| Observation | **45** | **232** (45 proprioceptive + 187 height scan) |
| Action | 12 joint position targets | same |
| Trained on | flat / cobblestone | 9-level cubic-stepfield curriculum |
| Result | velocity tracking error 0.54 m/s | terrain level 6.12 of 8 (≈13 cm cubes) |
| Robot model | Unitree `go2.usd` + `UnitreeActuator` | same |
| Domain randomisation | actuator latency, kp/kd error, IMU bias, friction | friction only |

Both drop `base_lin_vel` from the observation, so neither needs body linear velocity —
a quantity a real Go2 cannot measure in low-level mode. Everything they consume is
available from joint encoders, an IMU, the commanded velocity, and (for `rough`) a depth
camera.

---

## Values that MUST match — shared by both

| | |
|---|---|
| Control rate | **50 Hz** (`step_dt` 0.02 s) |
| Action scale | **0.25** |
| Action offset | the default joint pose below (action → target = `default + 0.25 * action`) |
| Joint stiffness / damping | **kp 25.0, kd 0.5** (all 12 joints) |
| Default joint pose | `[0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5]` |

### Joint ordering

The pose above and every joint-indexed observation are in **Isaac Lab order**, which
groups by joint type:

```
0 FL_hip   1 FR_hip   2 RL_hip   3 RR_hip
4 FL_thigh 5 FR_thigh 6 RL_thigh 7 RR_thigh
8 FL_calf  9 FR_calf 10 RL_calf 11 RR_calf
```

If your simulator uses **Unitree SDK order** (FR, FL, RR, RL; hip/thigh/calf per leg),
`deploy.yaml`'s `joint_ids_map` is the remap:

```
joint_ids_map = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
```

Read it as: `sdk_slot = joint_ids_map[isaac_index]`. Getting this wrong does not crash —
it produces a robot that moves confidently and wrongly.

---

## Observation layout

Concatenate the terms **in this exact order**. A reordered observation is the right width
and silently wrong, which is the failure this contract exists to prevent. The order is
also recorded explicitly as `observation_order` in each `deploy.yaml`.

### flat — 45 dims

| # | term | width | scale | source |
|---|---|---|---|---|
| 1 | `base_ang_vel` | 3 | **×0.2** | IMU gyro, body frame |
| 2 | `projected_gravity` | 3 | ×1.0 | gravity direction in body frame; ≈ (0,0,−1) upright |
| 3 | `velocity_commands` | 3 | ×1.0 | commanded (vx, vy, yaw rate) |
| 4 | `joint_pos_rel` | 12 | ×1.0 | joint position − default pose |
| 5 | `joint_vel_rel` | 12 | **×0.05** | joint velocity |
| 6 | `last_action` | 12 | ×1.0 | previous raw policy output, pre-scale |

Every term additionally clipped to (−100, 100) **before** scaling.

> **The two scales are not optional.** Feeding raw gyro and raw joint velocity puts them
> 5× and 20× outside the training distribution. Measured consequence elsewhere in this
> project: actions saturating at |a| = 28, a 7 rad joint-target offset.

Command envelope sampled during training: **±1.5 m/s** in x and y, ±1.5 rad/s yaw. Clamp
your command to that; the policy has never seen anything outside it.

### rough — 232 dims

Same six terms, **all at scale 1.0 and unclipped** (this policy descends from stock Isaac
Lab, which sets no scales), then:

| # | term | width | scale | clip |
|---|---|---|---|---|
| 7 | `height_scan` | **187** | ×1.0 | (−1.0, 1.0) |

Command envelope: **±1.0** on all three axes.

> Note the difference: `flat` scales `base_ang_vel` by 0.2 and `joint_vel_rel` by 0.05;
> `rough` scales neither. Do not share one observation builder between them without
> reading the scales out of each contract.

---

## The height scan (`rough` only)

187 values, a **17 × 11** grid of terrain heights sampled ahead of the robot.

| | |
|---|---|
| Grid size | 1.6 m (x, forward) × 1.0 m (y, left) |
| Resolution | 0.1 m |
| Centre | **0.6 m ahead** of the robot base origin, 0 m lateral |
| Frame | robot base — x forward, y left, z up |
| Value | `clip(sensor_z − hit_z − 0.5, −1.0, 1.0)` |
| Flatten order | **x fastest**: `heights[iy * 17 + ix]` |
| Unobserved cells | **0.0** — see the warning below |

`sensor_z − hit_z − 0.5` means a settled Go2 over flat ground reads about **−0.15**, a
15 cm step about −0.30, and a 30 cm cube about −0.45. Do not offset or clip a second time.

Re-sorting the flatten order scrambles the terrain view without changing the vector's
width, so it will not raise an error anywhere.

### ⚠ Unobserved cells must be 0.0, not −1.0

The policy was trained with a **forward-facing camera mask**: only the 97 of 187 cells a
RealSense D435i could actually see carry real heights, and the other 90 are filled with a
fixed token. For **this** policy that token is **0.0**.

The training project changed that constant to −1.0 on 2026-09-18, *after* this policy was
trained. If you are reading newer code from the same repo, or feeding a height map from a
newer `heightmap_node`, you will get −1.0 and corrupt roughly half the observation. The
authoritative value is in `rough/params/deploy.yaml`:

```yaml
observations:
  height_scan:
    params:
      unobserved_value: 0.0
```

If your simulator has full ground-truth terrain and no camera occlusion, you have two
options. Either reproduce the camera mask (cells outside a 89.44° × 58.24° FOV from a
camera at (0.28, 0, 0.10) m pitched 30° down, usable range 0.3–3.0 m, get 0.0), or feed
the true height everywhere and accept a distribution shift — the policy has never seen
real values in those cells. The first is faithful; the second is easier and will probably
degrade behaviour on the approach to obstacles, where the mask matters most.

---

## Importing

```python
import onnxruntime as ort, numpy as np, yaml

sess = ort.InferenceSession("rough/policy.onnx", providers=["CPUExecutionProvider"])
cfg  = yaml.safe_load(open("rough/params/deploy.yaml"))

obs = np.concatenate([...]).astype(np.float32)          # terms in cfg["observation_order"]
action = sess.run(None, {"obs": obs[None, :]})[0][0]    # (12,) raw output
target = np.array(cfg["default_joint_pos"]) + 0.25 * action
```

Input tensor is named `obs`, shape `(1, 45)` or `(1, 232)`; output `actions`, shape
`(1, 12)`. The ONNX is the policy **mean** — deterministic, no sampling, and unbounded, so
clamp the target if your actuators need protection. `policy.pt` is the same network as a
TorchScript module if you would rather not use onnxruntime.

Sanity check before trusting anything: hold the robot at the default pose with a zero
command and confirm the action stays small (|a| ≲ 1–2). Sustained large actions mean the
observation is wrong — almost always the joint order, a missing scale, or the scan token.

---

## Known limitations

- **Neither policy clears the RoboCup spec stepfields.** `rough` reaches terrain level
  6.12, roughly 13 cm cubes. The 15 cm and 30 cm spec rows were trained against but not
  mastered. Expect it to struggle above ~15 cm.
- **`rough` has weaker velocity tracking than `flat`** (0.64 vs 0.54 m/s error) — an
  honest trade for negotiating terrain, not a defect.
- **Neither has been validated on physical hardware.** Both are sim-trained and
  sim-verified only.
- `flat` carries actuator-latency and gain domain randomisation; `rough` does not, so
  `rough` assumes a more idealised actuator response.
