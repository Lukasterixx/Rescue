"""Export a K-Rail Square checkpoint (`go2_rescue/krail_env_cfg.py`) as a bundle the rescue sim walks with `--policy`.

    python training/export_krail.py RUN_DIR [--checkpoint model_5999.pt] --out policies/go2_krail_<date>

writes, in the layout of `policies/go2_policies_2026-09-18/` (whose README says what each file means):

    OUT/policy.pt               TorchScript: observation (1, N) -> raw action (1, 12), the policy mean, clamped at the
                                run's `clip_actions` (+-20) as training's RslRlVecEnvWrapper clamped it
    OUT/policy.onnx             the same network; input `obs`, output `actions`
    OUT/params/deploy.yaml      the deploy contract (`rescue_sim/deploy_contract.py` reads it)
    OUT/params/env.yaml         the run's own dump, which the sim reads the height scanner from
    OUT/params/agent.yaml       the run's own dump
    OUT/params/export.json      which run and checkpoint, the files' sha256, and how far the TorchScript and ONNX
                                outputs are from the rebuilt network

The contract is generated from the run's env.yaml, never typed: the control period, the leg order and default pose,
the gains, the action scale, the command envelope and every observation term's width, scale, clip and history. The
sim then checks the legs, gains, rate and width against its own robot before the policy gets them
(`rescue_sim/sim.py`, `_load_exported`), so a wrong value here stops the sim rather than walking it badly.

No Isaac Sim: torch and yaml only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

import torch
import yaml

# Isaac Lab's order for the Go2's legs, and the SDK remap (as `rescue_sim/exported_policy.py` and the 2026-09-18
# bundles have them).
ISAAC_LAB_LEG_ORDER = (
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
)
JOINT_IDS_MAP = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
LEG_GROUP = "base_legs"

# Widths of the stock observation functions this task uses; the scan's comes from its pattern.
TERM_WIDTHS = {"base_lin_vel": 3, "base_ang_vel": 3, "projected_gravity": 3, "generated_commands": 3,
               "joint_pos_rel": 12, "joint_vel_rel": 12, "last_action": 12}


class _EnvYamlLoader(yaml.SafeLoader):
    """Isaac Lab's env.yaml without running it: tuples are tuples, other Python tags are None."""


_EnvYamlLoader.add_constructor("tag:yaml.org,2002:python/tuple", lambda l, n: tuple(l.construct_sequence(n)))
_EnvYamlLoader.add_multi_constructor("tag:yaml.org,2002:python/", lambda l, s, n: None)


class KRailPolicy(torch.nn.Module):
    """The actor's mean, clamped as training clamped the action before the environment saw it."""

    def __init__(self, mlp: torch.nn.Sequential, clip: float):
        super().__init__()
        self.mlp = mlp
        self.clip = clip

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return torch.clamp(self.mlp(obs), -self.clip, self.clip)


def actor_mlp(state: dict, activation: str) -> torch.nn.Sequential:
    """rsl-rl 5's `MLPModel.mlp` rebuilt from `actor_state_dict`: Linear layers at even indices, the activation
    between them, none after the last. Fails if the checkpoint normalised its observations (this task does not)."""
    if any(k.startswith("obs_normalizer.") for k in state):
        raise SystemExit("the actor normalises its observations; this exporter does not carry a normaliser")
    indices = sorted(int(m.group(1)) for k in state if (m := re.fullmatch(r"mlp\.(\d+)\.weight", k)))
    act = {"elu": torch.nn.ELU}[activation]
    layers = []
    for n, i in enumerate(indices):
        w, b = state[f"mlp.{i}.weight"], state[f"mlp.{i}.bias"]
        linear = torch.nn.Linear(w.shape[1], w.shape[0])
        linear.weight.data.copy_(w)
        linear.bias.data.copy_(b)
        layers.append(linear)
        if n < len(indices) - 1:
            layers.append(act())
    return torch.nn.Sequential(*layers)


def resolve(pattern_values: dict, names) -> list[float]:
    """Isaac Lab's regex -> value dict resolved for each name (exactly one pattern must match each)."""
    out = []
    for name in names:
        hits = [v for p, v in pattern_values.items() if re.fullmatch(p, name)]
        if len(hits) != 1:
            raise SystemExit(f"{name}: {len(hits)} patterns in {pattern_values} match it")
        out.append(float(hits[0]))
    return out


def deploy_contract(env: dict) -> dict:
    robot = env["scene"]["robot"]
    legs = robot["actuators"][LEG_GROUP]
    action = env["actions"]["joint_pos"]
    if not action.get("use_default_offset"):
        raise SystemExit("the action term does not add the default pose; the sim assumes it does")
    if action.get("clip") is not None:
        raise SystemExit("the action term clips its targets; nothing here records that for the sim")
    ranges = env["commands"]["base_velocity"]["ranges"]
    scan = env["scene"]["height_scanner"]["pattern_cfg"]
    nx = int(round(scan["size"][0] / scan["resolution"])) + 1
    ny = int(round(scan["size"][1] / scan["resolution"])) + 1
    observations, order = {}, []
    for name, term in env["observations"]["policy"].items():
        if not isinstance(term, dict) or "func" not in term:
            continue
        func = term["func"].rsplit(":", 1)[-1]
        width = nx * ny if func == "height_scan" else TERM_WIDTHS[func]
        if term.get("scale") is not None:
            raise SystemExit(f"{name} is scaled in training; add its scale to the contract before exporting it")
        history = int(term.get("history_length") or 1)
        params = {"command_name": term["params"]["command_name"]} if func == "generated_commands" else {}
        observations[name] = {"params": params, "clip": list(term["clip"]) if term.get("clip") else None,
                              "scale": [1.0] * width, "history_length": history}
        order.append(name)
    return {
        "joint_ids_map": JOINT_IDS_MAP,
        "step_dt": float(env["sim"]["dt"]) * int(env["decimation"]),
        "stiffness": [float(legs["stiffness"])] * 12,
        "damping": [float(legs["damping"])] * 12,
        "default_joint_pos": resolve(robot["init_state"]["joint_pos"], ISAAC_LAB_LEG_ORDER),
        "commands": {"base_velocity": {"ranges": {
            "lin_vel_x": list(ranges["lin_vel_x"]), "lin_vel_y": list(ranges["lin_vel_y"]),
            "ang_vel_z": list(ranges["ang_vel_z"])}}},
        "actions": {"joint_pos": {"clip": None, "joint_names": list(action["joint_names"]),
                                  "scale": [float(action["scale"])] * 12, "offset": 0.0, "joint_ids": None}},
        "observations": observations,
        "observation_order": order,
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run", type=Path, help="the rsl_rl run directory (holds params/ and model_*.pt)")
    parser.add_argument("--checkpoint", default=None, help="model file in the run; default the latest")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    run = args.run.expanduser().resolve()
    models = sorted(run.glob("model_*.pt"), key=lambda p: int(p.stem.split("_")[1]))
    ckpt = run / args.checkpoint if args.checkpoint else models[-1]
    env = yaml.load((run / "params" / "env.yaml").read_text(), Loader=_EnvYamlLoader)
    agent = yaml.load((run / "params" / "agent.yaml").read_text(), Loader=_EnvYamlLoader)

    contract = deploy_contract(env)
    obs_dim = sum(len(t["scale"]) * t["history_length"] for t in contract["observations"].values())
    state = torch.load(ckpt, map_location="cpu", weights_only=False)["actor_state_dict"]
    mlp = actor_mlp(state, agent["actor"]["activation"])
    if mlp[0].in_features != obs_dim or mlp[-1].out_features != 12:
        raise SystemExit(f"the actor maps {mlp[0].in_features} -> {mlp[-1].out_features}; the contract says "
                         f"{obs_dim} -> 12")
    clip = float(agent["clip_actions"]) if agent.get("clip_actions") is not None else float("inf")
    policy = KRailPolicy(mlp, clip).eval()

    out = args.out.expanduser().resolve()
    (out / "params").mkdir(parents=True, exist_ok=True)
    scripted = torch.jit.script(policy)
    scripted.save(str(out / "policy.pt"))
    torch.onnx.export(policy, torch.zeros(1, obs_dim), str(out / "policy.onnx"), input_names=["obs"],
                      output_names=["actions"], dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
                      opset_version=17, dynamo=False)
    (out / "params" / "deploy.yaml").write_text(yaml.safe_dump(contract, sort_keys=False, width=120,
                                                               default_flow_style=None))
    shutil.copyfile(run / "params" / "env.yaml", out / "params" / "env.yaml")
    shutil.copyfile(run / "params" / "agent.yaml", out / "params" / "agent.yaml")

    # The TorchScript and the ONNX file must both be the rebuilt network (onnx's reference evaluator, since the
    # conda env has no onnxruntime).
    import numpy as np
    import onnx
    from onnx.reference import ReferenceEvaluator

    x = torch.randn(64, obs_dim) * 2.0
    with torch.no_grad():
        want = policy(x)
        err = (torch.jit.load(str(out / "policy.pt"))(x) - want).abs().max().item()
    model = onnx.load(str(out / "policy.onnx"))
    onnx.checker.check_model(model)
    onnx_err = float(np.abs(ReferenceEvaluator(model).run(None, {"obs": x.numpy()})[0] - want.numpy()).max())
    record = {
        "run": str(run), "checkpoint": ckpt.name, "iteration": int(ckpt.stem.split("_")[1]),
        "obs_dim": obs_dim, "action_clip": clip, "torchscript_vs_rebuilt_max_abs": err,
        "onnx_vs_rebuilt_max_abs": onnx_err,
        "sha256": {p.name: sha256(p) for p in (ckpt, out / "policy.pt", out / "policy.onnx")},
    }
    (out / "params" / "export.json").write_text(json.dumps(record, indent=1) + "\n")
    print(json.dumps(record, indent=1))


if __name__ == "__main__":
    main()
