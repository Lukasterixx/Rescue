"""The fixed, seeded benchmark every stair experiment is judged against.

WHY IT EXISTS. `measure_climb.py` could not rank checkpoints: run twice on the same
checkpoint it reported 12.5 % and 23.4 % of robots clearing 10 cm. It plays the rough PLAY
task, whose tiles each draw a random terrain type and difficulty from an unseeded generator,
so every run walked its robots over different ground with a handful on any one stair. This
probe removes every source of that variance it can:

  * THE GROUND IS FIXED AND IS THE MAZE'S: 10 cm risers on 20 cm treads (terrain_cfg.py's
    MazeCfg). Five columns, identical every run: flat, a 4-step flight up and down (the maze's
    longest), and an 11-step flight up and down. The flights are the training generator's own
    pyramid stairs, so the geometry is known exactly and every footfall is located against it.
  * EVERY ROBOT MEETS THE FLIGHT THE SAME WAY: facing it (yaw 0; the rings are square, so +x
    meets risers head-on), 1.2 m before the first riser or nosing, clear of the rings' corners.
  * THE ROBOT IS THE SIM'S. Base-mass and CoM randomisation are off, because the sim applies
    neither -- and that turned out to matter more than anything else here: training shifts the
    CoM 0..+5 cm forward, and the policy's forward speed follows that offset almost linearly
    (80 % tracking at 0 cm, 103 % at +2.5, 116 % at +5). Every earlier probe ran on the PLAY
    task with that randomisation still on, so none of them measured the robot the sim drives.
    `--randomize_mass`, `--mass_add` and `--com_x` put it back for comparison.
  * Friction pinned at 0.8/0.6, pushes and observation noise off, and `--seed` seeds the env.

GPU physics is not bit-exact, so the same seed still differs slightly between runs; run a
second seed before trusting a small difference between two checkpoints.

THE CLOCK, all robots together: stand 2 s, forward at `--speed` for 8 s (a stair robot is
stopped once its base is on the far landing), stop 3 s, then 3 s turning in place (flat only).

WHAT IT REPORTS
  command response   rise time to 50/90 % of the robot's own steady speed, where that steady
                     speed sits against the command, overshoot, backward motion after a forward
                     request, stopping time and distance
  touchdown          each foot against its own thigh joint at touchdown and lift-off (yaw
                     frame), the stance centre, the offset from Raibert's neutral point
                     v*T_stance/2, same-side front/rear spacing, stance, air time, stride
  stability          pitch-rate RMS, pitch wobble about a 0.5 s moving average, foot slip
  stairs             share reaching the first step, a 4-step flight and the far landing, falls,
                     time per step, stalls, and where the unfinished robots ended up
  footfalls          every stance on a flight classified as resting mid-tread, at the tread's
                     back or front edge, or on a vertical face / nosing corner, plus the share
                     with a mostly horizontal contact force
  turn               yaw tracking and drift at 1.0 rad/s, so a stair gain that costs the turn
                     shows in the same report

Contact edges come from the foot force (> 1 N) at control rate, not from the contact sensor's
compute_first_air(), which misses most lift-offs when read once per control step. Contacts
shorter than 60 ms are reported as chatter and kept out of every other number.

    cd ~/IsaacLab && ./isaaclab.sh -p ~/Rescue/Isaac/go2_omniverse/training/measure_bench.py \
      --checkpoint ~/IsaacLab/logs/rsl_rl/go2_rescue/<run>/model_XXXX.pt \
      --speed 0.5 --seed 1 --headless --out bench.txt --json bench.json
"""

import argparse
import json
import math
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rescue-Unitree-Go2-Play-v0")
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--robots_per_terrain", type=int, default=100)
parser.add_argument("--speed", type=float, default=0.5)
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--out", type=str, default="bench.txt")
parser.add_argument("--json", type=str, default=None, help="also write the metrics as JSON")
parser.add_argument("--turn_rate", type=float, default=1.0, help="yaw rate for the turn-in-place phase, rad/s")
parser.add_argument("--flat_only", action="store_true", help="only the flat column (for flat-ground policies)")
parser.add_argument("--clip_actions", type=float, default=20.0,
                    help="clamp policy actions before the env, as the experiment runner does; <= 0 disables")
parser.add_argument("--no_obs_patch", action="store_true", help="diagnostic: do not patch the command into the obs")
parser.add_argument("--randomize_mass", action="store_true", help="diagnostic: keep the task's base mass/CoM randomisation")
parser.add_argument("--mass_add", type=float, default=None, help="diagnostic: fixed kg added to the base")
parser.add_argument("--com_x", type=float, default=None, help="diagnostic: fixed base CoM x offset, m")
parser.add_argument("--dump", type=str, default=None, help="also save the raw event tensors (torch.save)")
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
sys.argv = [sys.argv[0]]
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import isaaclab.terrains as terrain_gen
from isaaclab.terrains import TerrainGeneratorCfg
from isaaclab.utils.math import euler_xyz_from_quat, quat_apply_inverse, yaw_quat
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
import importlib.metadata as _md
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

RISER = 0.10
TREAD = 0.20
TILE = 8.0
TILE_BORDER = 1.0
FLIGHT = 4                  # the maze's longest flight, 2-4 steps (maze_terrain.py)
APPROACH = 1.2              # spawn this far (base) before the first riser or nosing
LOADED_N = 1.0              # foot force that counts as contact, the contact sensor's own threshold
MIN_STANCE = 0.06           # contacts shorter than this are chatter, not footfalls

FEET = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
THIGHS = ["FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh"]

T_STAND, T_MOVE, T_STOP, T_TURN = 2.0, 8.0, 3.0, 3.0


def stair_geometry(platform):
    """(steps including the tile border, platform half-width), the generator's arithmetic.

    Replicated float for float: `(8 - 2 - 2) // 0.4` is 9.0 in IEEE doubles, not 10, so a
    nominal 2 m platform comes out as ten rings and a 2.0 m platform rather than eleven and 1.6.
    """
    inner = TILE - 2 * TILE_BORDER
    rings = int((inner - platform) // (2 * TREAD) + 1)
    return rings + 1, 0.5 * (inner - 2 * rings * TREAD)


# column -> (name, generator cfg class or None, sign of each step, platform width)
TERRAINS = [
    ("flat", None, 0.0, None),
    ("up4", terrain_gen.MeshInvertedPyramidStairsTerrainCfg, 1.0, 5.0),
    ("down4", terrain_gen.MeshPyramidStairsTerrainCfg, -1.0, 5.0),
    ("up11", terrain_gen.MeshInvertedPyramidStairsTerrainCfg, 1.0, 2.0),
    ("down11", terrain_gen.MeshPyramidStairsTerrainCfg, -1.0, 2.0),
]


TURN_RATE = args_cli.turn_rate


def main():
    if args_cli.flat_only:
        del TERRAINS[1:]
    nT = len(TERRAINS)
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=nT * args_cli.robots_per_terrain)
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, _md.version("rsl-rl-lib"))
    env_cfg.seed = args_cli.seed

    subs = {}
    for name, cls, _, platform in TERRAINS:
        if cls is None:
            subs[name] = terrain_gen.MeshPlaneTerrainCfg(proportion=1.0)
        else:
            subs[name] = cls(proportion=1.0, step_height_range=(RISER, RISER), step_width=TREAD,
                             platform_width=platform, border_width=TILE_BORDER, holes=False)
    # curriculum layout: one row, and each column is one sub-terrain in declaration order
    env_cfg.scene.terrain.terrain_generator = TerrainGeneratorCfg(
        seed=args_cli.seed, size=(TILE, TILE), border_width=20.0, num_rows=1, num_cols=nT,
        horizontal_scale=0.1, vertical_scale=0.005, slope_threshold=0.75, use_cache=False,
        curriculum=True, sub_terrains=subs)
    env_cfg.scene.terrain.max_init_terrain_level = 0
    env_cfg.curriculum.terrain_levels = None
    if hasattr(env_cfg.curriculum, "lin_vel_cmd_levels"):
        env_cfg.curriculum.lin_vel_cmd_levels = None

    # a nominal robot on a pinned surface
    if not args_cli.randomize_mass:
        if args_cli.mass_add is None:
            env_cfg.events.add_base_mass = None
        else:
            env_cfg.events.add_base_mass.params["mass_distribution_params"] = (args_cli.mass_add, args_cli.mass_add)
        if args_cli.com_x is None:
            env_cfg.events.base_com = None
        else:
            env_cfg.events.base_com.params["com_range"] = {"x": (args_cli.com_x, args_cli.com_x),
                                                           "y": (0.0, 0.0), "z": (0.0, 0.0)}
    env_cfg.events.push_robot = None
    env_cfg.events.base_external_force_torque = None
    env_cfg.observations.policy.enable_corruption = False
    env_cfg.events.physics_material.params.update({
        "static_friction_range": (0.8, 0.8), "dynamic_friction_range": (0.6, 0.6),
        "restitution_range": (0.0, 0.0)})
    env_cfg.events.reset_base.params = {
        "pose_range": {"x": (-0.1, 0.1), "y": (-0.3, 0.3), "yaw": (0.0, 0.0)},
        "velocity_range": {k: (0.0, 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
    }
    env_cfg.events.reset_robot_joints.params["velocity_range"] = (0.0, 0.0)

    cmd_cfg = env_cfg.commands.base_velocity
    cmd_cfg.heading_command = False
    cmd_cfg.rel_standing_envs = 0.0
    if hasattr(cmd_cfg, "rel_turning_envs"):
        cmd_cfg.rel_turning_envs = 0.0
    cmd_cfg.resampling_time_range = (1.0e6, 1.0e6)
    cmd_cfg.debug_vis = False
    env_cfg.episode_length_s = 3.0 * (T_STAND + T_MOVE + T_STOP + T_TURN)

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=args_cli.clip_actions if args_cli.clip_actions > 0 else None)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(retrieve_file_path(args_cli.checkpoint))
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    uenv = env.unwrapped
    dev = uenv.device
    N = uenv.num_envs
    dt = uenv.step_dt
    robot = uenv.scene["robot"]
    contacts = uenv.scene["contact_forces"]
    foot_ids, _ = robot.find_bodies(FEET, preserve_order=True)
    thigh_ids, _ = robot.find_bodies(THIGHS, preserve_order=True)
    cfoot_ids, _ = contacts.find_bodies(FEET, preserve_order=True)
    vel_cmd = uenv.command_manager.get_command("base_velocity")

    # -- per-robot geometry, and the spawn moved to APPROACH before each flight ---------------
    ttype = uenv.scene.terrain.terrain_types.clone()
    centre = uenv.scene.terrain.env_origins.clone()          # tile centre, z = platform top
    sign = torch.zeros(N, device=dev)
    half_plat = torch.full((N,), 1.0e3, device=dev)
    n_steps = torch.zeros(N, dtype=torch.long, device=dev)
    for c, (name, cls, sg, platform) in enumerate(TERRAINS):
        m = ttype == c
        if cls is not None:
            ns, hp = stair_geometry(platform)
            sign[m], half_plat[m], n_steps[m] = sg, hp, ns
            uenv.scene.terrain.env_origins[m, 0] = centre[m, 0] + hp - APPROACH
    is_flat = ttype == 0
    is_stair = ~is_flat
    half_inner = 0.5 * (TILE - 2 * TILE_BORDER)
    om = uenv.observation_manager
    layout, at = [], 0
    for tn, dims in zip(om.active_terms["policy"], om.group_obs_term_dim["policy"]):
        layout.append((tn, at, at + int(math.prod(dims))))
        at += int(math.prod(dims))
    cmd_slice = [(a_, b_) for tn, a_, b_ in layout if tn == "velocity_commands"]
    assert cmd_slice == [(9, 12)], f"command is not obs 9:12: {layout}"
    reset = env.reset()
    obs = reset[0] if isinstance(reset, tuple) else reset

    def steps_at(x_rel, ids):
        """Steps between the platform and the tread under tile-relative x (0 on the platform)."""
        hp = half_plat[ids]
        j = torch.floor((x_rel - hp) / TREAD) + 1
        j = torch.where(x_rel < hp, torch.zeros_like(j), j)
        return torch.minimum(j, n_steps[ids].float())

    v = args_cli.speed
    n_stand, n_move, n_stop, n_turn = (int(t / dt) for t in (T_STAND, T_MOVE, T_STOP, T_TURN))
    n_total = n_stand + n_move + n_stop + n_turn
    all_ids = torch.arange(N, device=dev)

    vx_tr = torch.zeros(n_total, N, device=dev)
    vy_tr = torch.zeros(n_total, N, device=dev)
    wz_tr = torch.zeros(n_total, N, device=dev)
    prate_tr = torch.zeros(n_total, N, device=dev)
    pitch_tr = torch.zeros(n_total, N, device=dev)
    x_tr = torch.zeros(n_total, N, device=dev)
    yaw_tr = torch.zeros(n_total, N, device=dev)
    scan_ahead_tr = torch.zeros(n_total, N, device=dev)
    scan_lo = None
    slip_sum = torch.zeros(N, device=dev)
    slip_n = torch.zeros(N, device=dev)
    slip_big = torch.zeros(N, device=dev)
    act_hist = torch.zeros(64, device=dev)          # |action| histogram, 0.25 bins from 0 to 16
    act_max = torch.zeros(12, device=dev)

    alive = torch.ones(N, dtype=torch.bool, device=dev)
    fell_at = torch.full((N,), -1, dtype=torch.long, device=dev)
    stair_done = torch.zeros(N, dtype=torch.bool, device=dev)
    done_at = torch.full((N,), -1, dtype=torch.long, device=dev)
    max_steps = int(n_steps.max().item())
    step_at = torch.full((N, max_steps + 1), -1, dtype=torch.long, device=dev)
    move_end = torch.full((N,), n_stand + n_move, dtype=torch.long, device=dev)

    # One row per COMPLETED stance, written at lift-off from what was stored at touchdown.
    # Isaac Lab's compute_first_air/compute_first_contact are not used: read once per control
    # step they miss most lift-offs (float32 timestamps against a 1e-8 tolerance), so contact
    # edges are taken from the foot force directly.
    F = ["t_td", "env", "foot", "dx_td", "dy_td", "gap", "fx", "fy", "z_s", "past", "ahead",
         "hfrac", "vx", "air", "stride", "dx_lo", "stance"]
    C = {n: i for i, n in enumerate(F)}
    NP = C["dx_lo"]                                   # fields known at touchdown
    pend = torch.full((N, 4, NP), float("nan"), device=dev)
    rows = []
    prev_loaded = torch.ones(N, 4, dtype=torch.bool, device=dev)
    last_lo = torch.full((N, 4), -1, dtype=torch.long, device=dev)
    last_td_xy = torch.full((N, 4, 2), float("nan"), device=dev)
    same_side = torch.tensor([2, 3, 0, 1], device=dev)
    foot_r = None

    with torch.inference_mode():
        for i in range(n_total):
            ph = 0 if i < n_stand else 1 if i < n_stand + n_move else 2 if i < n_stand + n_move + n_stop else 3
            cx = torch.zeros(N, device=dev)
            wz = torch.zeros(N, device=dev)
            if ph == 1:
                cx = torch.where(stair_done, cx, torch.full_like(cx, v))
            if ph == 3:
                wz = torch.where(is_flat, torch.full_like(wz, TURN_RATE), wz)
            vel_cmd[:, 0], vel_cmd[:, 1], vel_cmd[:, 2] = cx, 0.0, wz
            # The observation in hand carries LAST step's command; patch its command slice
            # (9:12, after lin vel, ang vel, gravity) so a change is seen on the step it happens.
            if not args_cli.no_obs_patch:
                pol = obs["policy"] if hasattr(obs, "keys") else obs
                pol[:, 9], pol[:, 10], pol[:, 11] = cx, 0.0, wz

            act = policy(obs)
            live_act = act[alive].abs()
            act_hist += torch.histc(live_act.clamp(max=15.99), bins=64, min=0.0, max=16.0)
            act_max = torch.maximum(act_max, live_act.max(0).values) if live_act.numel() else act_max
            obs, _, dones, _ = env.step(act)
            d = dones.bool()
            fell_at[alive & d] = i
            alive &= ~d

            root = robot.data.root_pos_w
            q = robot.data.root_quat_w
            lin_b, ang_b = robot.data.root_lin_vel_b, robot.data.root_ang_vel_b
            vx_tr[i], vy_tr[i], wz_tr[i], prate_tr[i] = lin_b[:, 0], lin_b[:, 1], ang_b[:, 2], ang_b[:, 1]
            _, pit, yw = euler_xyz_from_quat(q)
            yaw_tr[i] = torch.atan2(torch.sin(yw), torch.cos(yw))
            pitch_tr[i] = torch.atan2(torch.sin(pit), torch.cos(pit))

            # stair progress at the base
            rel = root - centre
            x_tr[i] = rel[:, 0] - half_plat
            pol_now = obs["policy"] if hasattr(obs, "keys") else obs
            # mean of the scan's front three longitudinal columns (grid 11 x 17, forward = high index)
            scan_ahead_tr[i] = pol_now[:, -187:].reshape(N, 11, 17)[:, :, -3:].mean((1, 2))
            j = steps_at(rel[:, 0], all_ids).long()
            for s_ in range(max_steps + 1):
                hit = is_stair & alive & (j >= s_) & (step_at[:, s_] < 0)
                step_at[hit, s_] = i
            fin = is_stair & alive & ~stair_done & (rel[:, 0] > half_inner + 0.35)
            stair_done |= fin
            done_at[fin] = i
            if ph == 1:
                move_end[fin] = i

            # feet
            yq4 = yaw_quat(q).repeat_interleave(4, 0)
            foot_w = robot.data.body_pos_w[:, foot_ids, :]
            thigh_w = robot.data.body_pos_w[:, thigh_ids, :]
            foot_hip = quat_apply_inverse(yq4, (foot_w - thigh_w).reshape(-1, 3)).reshape(N, 4, 3)
            foot_base = quat_apply_inverse(yq4, (foot_w - root[:, None, :]).reshape(-1, 3)).reshape(N, 4, 3)
            f = contacts.data.net_forces_w[:, cfoot_ids, :]
            loaded = f.norm(dim=-1) > LOADED_N
            speed = robot.data.body_lin_vel_w[:, foot_ids, :2].norm(dim=-1)
            live = alive & (ph == 1) & (i >= n_stand + int(1.5 / dt)) & ~stair_done
            slip_sum += torch.where(live, (speed * loaded).sum(1), 0.0)
            slip_n += torch.where(live, loaded.sum(1).float(), 0.0)
            slip_big += torch.where(live, ((speed > 0.1) & loaded).sum(1).float(), 0.0)

            if i == n_stand - 1:
                foot_r = (foot_w[is_flat, :, 2] - centre[is_flat, None, 2]).median().item()
                stand_dx = foot_hip[is_flat, :, 0].mean(0).tolist()

            td = loaded & ~prev_loaded & alive[:, None]
            lo = ~loaded & prev_loaded & alive[:, None]
            prev_loaded = loaded
            if i < n_stand:
                last_lo[lo] = i
                continue

            e, k = torch.nonzero(lo, as_tuple=True)
            if len(e):
                p = pend[e, k]
                ok = torch.isfinite(p[:, C["t_td"]])
                if ok.any():
                    stance = (i - p[:, C["t_td"]]) * dt
                    rows.append(torch.cat([p, foot_hip[e, k, 0:1], stance[:, None]], 1)[ok])
                pend[e, k] = float("nan")
                last_lo[e, k] = i

            e, k = torch.nonzero(td, as_tuple=True)
            if len(e):
                fx = foot_w[e, k, 0] - centre[e, 0]
                fy = foot_w[e, k, 1] - centre[e, 1]
                fz = foot_w[e, k, 2] - centre[e, 2]
                js = steps_at(fx, e)
                surf = sign[e] * js * RISER
                hp = half_plat[e]
                past = torch.where(js > 0, fx - (hp + (js - 1) * TREAD), torch.full_like(fx, 9.0))
                ahead = torch.where(js < n_steps[e].float(), (hp + js * TREAD) - fx, torch.full_like(fx, 9.0))
                fv = f[e, k]
                hfrac = fv[:, :2].norm(dim=-1) / fv.norm(dim=-1).clamp(min=1e-6)
                gap = torch.where(k < 2, foot_base[e, k, 0] - foot_base[e, same_side[k], 0],
                                  foot_base[e, same_side[k], 0] - foot_base[e, k, 0])
                air = torch.where(last_lo[e, k] >= 0, (i - last_lo[e, k]).float() * dt,
                                  torch.full_like(fx, float("nan")))
                stride = torch.linalg.norm(foot_w[e, k, :2] - last_td_xy[e, k], dim=-1)
                last_td_xy[e, k] = foot_w[e, k, :2]
                pend[e, k] = torch.stack([torch.full_like(fx, i), e.float(), k.float(),
                                          foot_hip[e, k, 0], foot_hip[e, k, 1], gap, fx, fy, fz - surf,
                                          past, ahead, hfrac, lin_b[e, 0], air, stride], 1)

    # ======================================================================== analysis ====
    R = torch.cat(rows) if rows else torch.zeros(0, len(F), device=dev)
    t_td, e_r, k_r = R[:, C["t_td"]].long(), R[:, C["env"]].long(), R[:, C["foot"]].long()
    front = k_r < 2
    real = R[:, C["stance"]] >= MIN_STANCE
    in_move = (t_td >= n_stand + int(1.5 / dt)) & (t_td < move_end[e_r])
    M, out = {}, []

    def pct(x):
        return 100.0 * x.float().mean().item() if x.numel() else float("nan")

    def mean(x):
        x = x[torch.isfinite(x)]
        return x.mean().item() if x.numel() else float("nan")

    def med(x):
        x = x[torch.isfinite(x)]
        return x.median().item() if x.numel() else float("nan")

    ck = args_cli.checkpoint.split("/")
    robot_desc = ("randomised mass and CoM, as in training" if args_cli.randomize_mass else
                  f"base +{args_cli.mass_add or 0.0:.2f} kg, CoM x {(args_cli.com_x or 0.0) * 100:+.1f} cm")
    out += [f"MEASURE_BENCH  {ck[-2]}/{ck[-1]}   robot: {robot_desc}",
            f"seed {args_cli.seed}, {args_cli.robots_per_terrain} robots per terrain, forward {v:.2f} m/s, "
            f"maze stairs {RISER * 100:.0f} cm riser x {TREAD * 100:.0f} cm tread, spawn {APPROACH} m before the flight",
            f"foot origin at rest {foot_r * 100:.2f} cm above flat ground (used as the foot radius)",
            "standing, flat: foot ahead of its thigh joint FL/FR/RL/RR "
            + " ".join(f"{x * 100:+.1f}" for x in stand_dx) + " cm", ""]

    # -- command response, flat -------------------------------------------------------------
    fl = is_flat & (fell_at < 0)
    on, stop0 = n_stand, n_stand + n_move
    ens = vx_tr[:, fl].mean(1)
    # every robot starts its gait on the same step, so the ensemble oscillates at stride
    # frequency; a centred 0.26 s window removes that without shifting the edge
    smooth = torch.nn.functional.avg_pool1d(ens[None, None], 13, 1, 6, count_include_pad=False)[0, 0]

    def first_cross(trace, start, level, above=True):
        idx = torch.nonzero(trace[start:] >= level if above else trace[start:] <= level)
        return idx[0].item() * dt if len(idx) else float("nan")

    steady = ens[on + int(4.0 / dt): stop0].mean().item()
    # rise times against the speed the robot settles at, so one that tracks 80 % still gets a
    # rise time; how far short it settles is `steady`
    t50, t90 = first_cross(smooth, on, 0.5 * steady), first_cross(smooth, on, 0.9 * steady)
    overshoot = (smooth[on:stop0].max().item() / max(steady, 1e-6) - 1) * 100
    early = vx_tr[on: on + int(1.0 / dt), fl]
    back_min = early.min(0).values
    back_dist = torch.cumsum(early * dt, 0).min(0).values
    t_stop = first_cross(smooth.abs(), stop0, 0.1 * v, above=False)
    stop_dist = (vx_tr[stop0: stop0 + n_stop, fl] * dt).sum(0)
    creep = vx_tr[stop0 + int(1.5 / dt): stop0 + n_stop, fl].abs().mean().item()
    M["response"] = dict(t50=t50, t90=t90, overshoot=overshoot, steady_pct=steady / v * 100,
                         backward_pct=pct(back_min < -0.05), backward_worst=mean(back_min),
                         backward_dist_cm=mean(back_dist) * 100, t_stop=t_stop,
                         stop_dist_cm=mean(stop_dist) * 100, creep=creep)
    r = M["response"]
    out += ["COMMAND RESPONSE, flat ground, ensemble mean of body-frame vx",
            f"  0 -> {v:.2f} m/s: 50 % / 90 % of its own steady speed after {t50:.2f} / {t90:.2f} s, "
            f"overshoot {overshoot:+.1f} %, settles at {r['steady_pct']:.1f} % of the command",
            f"  moving backward within 1 s of the request: {r['backward_pct']:.1f} % of robots below -0.05 m/s "
            f"(mean worst {r['backward_worst']:+.3f} m/s, mean retreat {r['backward_dist_cm']:+.1f} cm)",
            f"  {v:.2f} -> 0: below 10 % of the command after {t_stop:.2f} s, travels {r['stop_dist_cm']:.1f} cm, "
            f"|vx| {creep:.3f} m/s once settled",
            "  vx every 0.25 s from the request: "
            + " ".join(f"{smooth[on + int(0.25 * s_ / dt)].item():.2f}" for s_ in range(13)), ""]

    t0 = n_stand + n_move + n_stop + int(1.0 / dt)
    M["turn"] = dict(pct=wz_tr[t0:, fl].mean().item() / TURN_RATE * 100,
                     drift=torch.sqrt(vx_tr[t0:, fl] ** 2 + vy_tr[t0:, fl] ** 2).mean().item())
    out += [f"TURN IN PLACE {TURN_RATE:.1f} rad/s, flat: tracking {M['turn']['pct']:.1f} %, "
            f"drift {M['turn']['drift']:.3f} m/s", ""]

    names = [t[0] for t in TERRAINS]

    # -- touchdown -------------------------------------------------------------------------------
    out += ["TOUCHDOWN during the forward command (first 1.5 s excluded; stair robots until the far landing)",
            "  td / lo   foot ahead of its own thigh joint at touchdown / lift-off, yaw frame, + forward",
            "  mid       centre of the stance sweep; 0 = stance centred under the hip",
            "  td-neut   td minus Raibert's neutral point v*stance/2; 0 = the foot lands where the",
            "            stance would centre under the hip, negative = lands short",
            "  gap       front foot ahead of the same-side rear foot at the front touchdown (hips 38.7 cm apart)",
            "  chatter   contacts shorter than 60 ms, % of all contacts; excluded from every other column",
            f"{'':>13} {'n':>5} {'td':>6} {'lo':>6} {'mid':>6} {'stance':>7} {'air':>5} {'stride':>7} "
            f"{'v':>5} {'td-neut':>8} {'gap':>6} {'chatter':>8}",
            f"{'':>13} {'':>5} {'cm':>6} {'cm':>6} {'cm':>6} {'ms':>7} {'ms':>5} {'cm':>7} {'m/s':>5} "
            f"{'cm':>8} {'cm':>6} {'%':>8}"]
    for c, tname in enumerate(names):
        tm = ttype[e_r] == c
        for fname, fm in (("front", front), ("rear", ~front)):
            allc = in_move & tm & fm
            sel = allc & real
            td_ = mean(R[sel, C["dx_td"]]) * 100
            lo_ = mean(R[sel, C["dx_lo"]]) * 100
            st_ = mean(R[sel, C["stance"]])
            air_ = mean(R[sel, C["air"]]) * 1000
            sd = R[sel, C["stride"]]
            sd_ = mean(sd[sd < 1.0]) * 100
            v_ = mean(R[sel, C["vx"]])
            neut = td_ - v_ * st_ / 2 * 100
            gap_ = mean(R[sel, C["gap"]]) * 100 if fname == "front" else float("nan")
            chat = 100.0 * (allc & ~real).sum().item() / max(allc.sum().item(), 1)
            M[f"td_{tname}_{fname}"] = dict(n=int(sel.sum().item()), td_cm=td_, lo_cm=lo_, mid_cm=0.5 * (td_ + lo_),
                                            stance_ms=st_ * 1000, air_ms=air_, stride_cm=sd_, v=v_,
                                            td_minus_neutral_cm=neut, gap_cm=gap_, chatter_pct=chat)
            out.append(f"{tname + ' ' + fname:>13} {int(sel.sum().item()):5d} {td_:6.1f} {lo_:6.1f} "
                       f"{0.5 * (td_ + lo_):6.1f} {st_ * 1000:7.0f} {air_:5.0f} {sd_:7.1f} {v_:5.2f} {neut:8.1f} "
                       + (f"{gap_:6.1f}" if fname == "front" else f"{'':>6}") + f" {chat:8.1f}")
    out.append("")

    # -- stability ---------------------------------------------------------------------------------
    out += ["STABILITY during the forward command (same window)",
            "  wobble = pitch minus its own 0.5 s moving average, so a stair's slope and the entry and",
            "  exit are not counted",
            f"{'':>8} {'pitch-rate RMS':>15} {'wobble sd':>10} {'foot slip':>10} {'slipping':>9}",
            f"{'':>8} {'rad/s':>15} {'deg':>10} {'m/s':>10} {'% loaded':>9}"]
    a, b = n_stand + int(1.5 / dt), n_stand + n_move
    for c, tname in enumerate(names):
        sel = (ttype == c) & (fell_at < 0)
        valid = torch.arange(a, b, device=dev)[:, None] < move_end[None, sel]
        pr = prate_tr[a:b][:, sel][valid]
        pa = pitch_tr[a:b][:, sel].T.contiguous()
        pa_avg = torch.nn.functional.avg_pool1d(pa[:, None, :], 25, 1, 12, count_include_pad=False)[:, 0, :]
        wob = (pa - pa_avg).T[valid]
        prms = torch.sqrt((pr ** 2).mean()).item() if pr.numel() else float("nan")
        wsd = math.degrees(wob.std().item()) if wob.numel() > 1 else float("nan")
        slip = (slip_sum[sel].sum() / slip_n[sel].sum().clamp(min=1)).item()
        slipb = (slip_big[sel].sum() / slip_n[sel].sum().clamp(min=1)).item() * 100
        M[f"stab_{tname}"] = dict(pitch_rate_rms=prms, wobble_deg=wsd, slip=slip, slipping_pct=slipb)
        out.append(f"{tname:>8} {prms:15.3f} {wsd:10.2f} {slip:10.3f} {slipb:9.1f}")
    out.append("")

    # -- stairs ----------------------------------------------------------------------------------
    out += ["STAIRS, forward until the far landing. Progress is judged at the base.",
            "  stalled: more than 3 s without reaching a new step (on the flight, before finishing)",
            "  stuck at: for robots that did not finish, where the base ended, m past the first riser/nosing",
            f"{'':>8} {'steps':>5} {'fell':>6} {'1 step':>7} {'4 steps':>8} {'finish':>7} {'s/step':>7} "
            f"{'stalled':>8} {'stuck at':>9}",
            f"{'':>8} {'':>5} {'%':>6} {'%':>7} {'%':>8} {'%':>7} {'median':>7} {'%':>8} {'m median':>9}"]
    final_rel = robot.data.root_pos_w - centre
    for c, (tname, cls, _, _) in enumerate(TERRAINS):
        if cls is None:
            continue
        ids = torch.nonzero(ttype == c).flatten()
        ns = int(n_steps[ids[0]].item())
        fl_ = min(FLIGHT, ns)
        reached = step_at[ids]
        durs, stalled = [], 0
        for n_, env_i in enumerate(ids.tolist()):
            row = reached[n_]
            if row[1] >= 0 and row[fl_] >= 0:
                durs.append((row[fl_] - row[1]).item() * dt / max(fl_ - 1, 1))
            end = (done_at[env_i].item() if done_at[env_i] >= 0 else
                   fell_at[env_i].item() if fell_at[env_i] >= 0 else n_stand + n_move)
            first = row[1].item() if row[1] >= 0 else None
            if first is None:
                stalled += 1        # never reached the first step at all
                continue
            ev = sorted({first, end, *[x for x in row[1:ns + 1].tolist() if 0 <= x <= end]})
            if max(b_ - a_ for a_, b_ in zip(ev[:-1], ev[1:])) * dt > 3.0:
                stalled += 1
        unfinished = ids[~stair_done[ids] & (fell_at[ids] < 0)]
        stuck = med(final_rel[unfinished, 0] - half_plat[unfinished])
        st = dict(steps=ns, fell=pct(fell_at[ids] >= 0), one=pct(reached[:, 1] >= 0),
                  flight=pct(reached[:, fl_] >= 0), finish=pct(stair_done[ids]),
                  s_per_step=float(torch.tensor(durs).median()) if durs else float("nan"),
                  stalled=100.0 * stalled / len(ids), stuck_at_m=stuck)
        M[f"stairs_{tname}"] = st
        out.append(f"{tname:>8} {ns:5d} {st['fell']:6.1f} {st['one']:7.1f} {st['flight']:8.1f} "
                   f"{st['finish']:7.1f} {st['s_per_step']:7.2f} {st['stalled']:8.1f} {stuck:9.2f}")
    out.append("")

    tol, near = 0.015, foot_r + 0.01
    out += ["FOOTFALLS ON THE FLIGHT: every stance of 60 ms or more whose touchdown was between 25 cm",
            "before the first edge and 10 cm past the last, clear of the rings' corners. % of footfalls.",
            "  tread   supported on a tread, centre more than a radius + 1 cm from both its edges",
            "  back    supported, within a radius + 1 cm of the tread's BACK edge",
            "          (up: just past the nosing it stepped over; down: heel against the riser behind)",
            "  front   supported, within a radius + 1 cm of the tread's FRONT edge",
            "          (up: toe against the next riser; down: hanging over the nosing)",
            "  face    not resting on a tread at all: against a vertical face or on a nosing corner",
            "  hforce  contact force more than 70 % horizontal at touchdown (independent of geometry)",
            f"{'':>15} {'n':>5} {'tread':>6} {'back':>6} {'front':>6} {'face':>6} {'hforce':>7} "
            f"{'past back':>10} {'to front':>9}",
            f"{'':>15} {'':>5} {'%':>6} {'%':>6} {'%':>6} {'%':>6} {'%':>7} {'cm med':>10} {'cm med':>9}"]
    for c, (tname, cls, _, _) in enumerate(TERRAINS):
        if cls is None:
            continue
        for fname, fm in (("front", front), ("rear", ~front), ("all", torch.ones_like(front))):
            fx_ = R[:, C["fx"]]
            sel = ((ttype[e_r] == c) & fm & real & (t_td >= n_stand) & (fx_ > half_plat[e_r] - 0.25)
                   & (fx_ < half_inner + 0.10) & (R[:, C["fy"]].abs() < fx_ - 0.15))
            Rs = R[sel]
            sup = (Rs[:, C["z_s"]] - foot_r).abs() <= tol
            back = sup & (Rs[:, C["past"]] < near)
            frnt = sup & ~back & (Rs[:, C["ahead"]] < near)
            cc = dict(n=int(sel.sum().item()), tread=pct(sup & ~back & ~frnt), back=pct(back), front=pct(frnt),
                      face=pct(~sup), hforce=pct(Rs[:, C["hfrac"]] > 0.7),
                      past_cm=med(Rs[sup & (Rs[:, C["past"]] < 1), C["past"]]) * 100,
                      ahead_cm=med(Rs[sup & (Rs[:, C["ahead"]] < 1), C["ahead"]]) * 100)
            M[f"footfall_{tname}_{fname}"] = cc
            out.append(f"{tname + ' ' + fname:>15} {cc['n']:5d} {cc['tread']:6.1f} {cc['back']:6.1f} "
                       f"{cc['front']:6.1f} {cc['face']:6.1f} {cc['hforce']:7.1f} {cc['past_cm']:10.1f} "
                       f"{cc['ahead_cm']:9.1f}")

    cdf = torch.cumsum(act_hist, 0) / act_hist.sum()
    q = lambda p_: (torch.nonzero(cdf >= p_)[0].item() + 1) * 0.25
    M["actions"] = dict(p99=q(0.99), p999=q(0.999), p9999=q(0.9999), max_per_joint=act_max.tolist())
    out += ["", f"ACTIONS (policy mean, all robots and steps): |a| p99 {q(0.99):.2f}, p99.9 {q(0.999):.2f}, "
            f"p99.99 {q(0.9999):.2f}; before the clip at {args_cli.clip_actions}, max per joint " + " ".join(f"{x:.1f}" for x in act_max.tolist())]
    if args_cli.dump:
        torch.save(dict(R=R.cpu(), F=F, ttype=ttype.cpu(), final_rel=final_rel.cpu(), fell_at=fell_at.cpu(),
                        step_at=step_at.cpu(), vx=vx_tr.cpu(), x=x_tr.cpu(), yaw=yaw_tr.cpu(), scan_ahead=scan_ahead_tr.cpu(), pitch=pitch_tr.cpu(), n_stand=n_stand,
                        n_move=n_move, dt=dt, move_end=move_end.cpu(), half_plat=half_plat.cpu()),
                   args_cli.dump)
    text = "\n".join(out)
    print(text)
    with open(args_cli.out, "w") as fh:
        fh.write(text + "\n")
    if args_cli.json:
        with open(args_cli.json, "w") as fh:
            json.dump(M, fh, indent=1)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
