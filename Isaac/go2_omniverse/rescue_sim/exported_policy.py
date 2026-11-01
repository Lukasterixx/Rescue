"""Walking policies exported with a deploy contract: `policy.pt` beside `params/deploy.yaml` and `params/env.yaml`, as
the Go2_RL_workflow project exports them (`../policies/go2_policies_2026-09-18/`, its flat and rough Go2 policies).

The sim's own policy is an rsl_rl checkpoint named in `agent_cfg.py`, read through the 235-wide observation in
`env_cfg.py`. An exported policy brings its own layout, so `--policy DIR` builds the observation from its contract
instead: the terms in `observation_order`, each with its recorded scale and clip, and the velocity command clamped
to the envelope it was sampled from. Nothing is transcribed. Every one of those values fails silently when wrong,
which is why the bundle's README asks for the contract to be read rather than copied, and why `deploy_contract.py`
(the bundle's reader, verbatim) refuses a contract it cannot honour.

The contract leaves out how the height scan was taken. `params/env.yaml`, written by the same training run, has the
rest:
* where the scanner sits. The rough policy's 17 x 11 grid is centred 0.6 m ahead of the base; the sim's own policy
  has it centred on the base.
* which function read it. Stock Isaac Lab's `height_scan` reads every cell. The rough policy's
  `height_scan_camera_fov` reads only the cells a forward RealSense could see and fills the rest with the contract's
  `unobserved_value`.

That function's code is not in the bundle, so `CameraMask` rebuilds it from the README's description: a D435i at
(0.28, 0, 0.10) m in the base frame, pitched 30 deg down, 89.44 x 58.24 deg, 0.3-3.0 m of depth. Two readings are
ours: the camera turns with the whole base (roll and pitch too, as a bolted-on camera does), and the range is depth
along the optical axis (what a D435 reports). On flat ground at a standing height it leaves 97 of the 187 cells
visible, as the README says, whichever range is used (`tests/test_exported_policy.py`).

A WHEELED POLICY (the Go2-W's, `../go2w.py`) drives four wheels too. Its deploy.yaml has a second action term,
`wheel_vel`, a velocity target per wheel after the legs' twelve, and a `wheels` section with the wheels' joints, SDK
indices and drive; its `joint_vel` term reads the wheels' speeds after the legs' (16 wide) and its last action is 16
wide. `deploy_contract.py` refuses a second action term ("the policy nodes drive joint positions only"), which is
right for a node that drives legs only, so `Bundle.load` takes the wheel term and the section off (`WheelContract`)
before the reader sees the rest, and checks them itself.

No Isaac Lab imports: the tests run this with the conda python outside Kit.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path

import torch
import yaml

from .deploy_contract import DeployContract, DeployContractError

# The order every joint-indexed term and the default pose are in: Isaac Lab's, grouped by joint type. The sim's leg
# terms must resolve to exactly this, or the policy moves confidently and wrongly (the bundle's README).
ISAAC_LAB_LEG_ORDER = (
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
)

# The Go2-W's wheels, in the order a wheeled policy's actions and wheel speeds are in (`go2w.WHEEL_JOINTS`).
WHEEL_ORDER = ("FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint")
WHEEL_ACTION_TERM = "wheel_vel"

# The terms the sim can build (`env_cfg.exported_observations`). deploy_contract knows two more, `gait_phase` and
# `posture_command`, which nothing here produces.
SIM_TERMS = ("base_lin_vel", "base_ang_vel", "projected_gravity", "velocity_commands", "joint_pos_rel",
             "joint_vel_rel", "last_action", "height_scan")

# The height-scan functions the sim reproduces, by the name env.yaml records.
STOCK_SCAN = "height_scan"
CAMERA_SCAN = "height_scan_camera_fov"


class ExportedPolicyError(RuntimeError):
    """The policy cannot be run faithfully here; the message says why."""


@dataclass(frozen=True)
class CameraMask:
    """The cells of the height scan a forward camera can see (see the module docstring for where the numbers come
    from). `pos` is in the base frame, x forward, y left, z up."""

    pos: tuple = (0.28, 0.0, 0.10)
    pitch_deg: float = 30.0
    hfov_deg: float = 89.44
    vfov_deg: float = 58.24
    near_m: float = 0.3
    far_m: float = 3.0

    def visible(self, hits_w: torch.Tensor, base_pos_w: torch.Tensor, base_quat_w: torch.Tensor) -> torch.Tensor:
        """(N, R) bool: which of the ray hits `hits_w` (N, R, 3) the camera sees, for bases at `base_pos_w` (N, 3)
        with orientation `base_quat_w` (N, 4, w first). A ray that hit nothing is not seen."""
        rel = hits_w - base_pos_w[:, None, :]
        d = _rotate_inverse(base_quat_w, rel) - torch.tensor(self.pos, dtype=hits_w.dtype, device=hits_w.device)
        pitch = math.radians(self.pitch_deg)
        # The optical axis is forward pitched down; the image's left is the base's left and its up is their cross.
        depth = d[..., 0] * math.cos(pitch) - d[..., 2] * math.sin(pitch)
        lateral = d[..., 1]
        vertical = d[..., 0] * math.sin(pitch) + d[..., 2] * math.cos(pitch)
        return (
            torch.isfinite(d).all(dim=-1)
            & (depth >= self.near_m) & (depth <= self.far_m)
            & (lateral.abs() <= depth * math.tan(math.radians(self.hfov_deg) / 2))
            & (vertical.abs() <= depth * math.tan(math.radians(self.vfov_deg) / 2))
        )


def _rotate_inverse(quat_wxyz: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate the vectors v (N, R, 3) by the inverse of the quaternions (N, 4): world to base frame."""
    w = quat_wxyz[:, None, 0:1]
    u = -quat_wxyz[:, None, 1:4].expand_as(v)
    t = 2.0 * torch.cross(u, v, dim=-1)
    return v + w * t + torch.cross(u, t, dim=-1)


def height_scan(base_pos_w: torch.Tensor, base_quat_w: torch.Tensor, hits_w: torch.Tensor, offset: float,
                mask: CameraMask | None, unobserved: float) -> torch.Tensor:
    """What the policy's height-scan function returns before its clip: `sensor_z - hit_z - offset` per cell. The
    sensor's z is the base's (Isaac Lab applies the scanner's offset to the rays only). With a mask, cells the
    camera cannot see, and rays that hit nothing, read `unobserved`; without one this is stock Isaac Lab's
    `height_scan`, misses included."""
    h = base_pos_w[:, 2:3] - hits_w[..., 2] - offset
    if mask is None:
        return h
    seen = mask.visible(hits_w, base_pos_w, base_quat_w) & torch.isfinite(h)
    return torch.where(seen, h, torch.full_like(h, float(unobserved)))


@dataclass(frozen=True)
class ScannerGeometry:
    """The training run's height scanner, from its env.yaml: a downward grid of rays from `offset` in the base's
    yaw frame."""

    offset: tuple
    resolution: float
    size: tuple
    ordering: str
    ray_alignment: str

    @property
    def num_rays(self) -> int:
        nx = int(round(self.size[0] / self.resolution)) + 1
        ny = int(round(self.size[1] / self.resolution)) + 1
        return nx * ny


class _EnvYamlLoader(yaml.SafeLoader):
    """Reads Isaac Lab's dumped env.yaml without running any of it: Python tuples become tuples and every other
    Python tag (slices, objects) becomes None."""


_EnvYamlLoader.add_constructor("tag:yaml.org,2002:python/tuple",
                               lambda loader, node: tuple(loader.construct_sequence(node)))
_EnvYamlLoader.add_multi_constructor("tag:yaml.org,2002:python/", lambda loader, suffix, node: None)


@dataclass(frozen=True)
class WheelContract:
    """A wheeled policy's wheels, from its deploy.yaml: the `wheel_vel` action term (a velocity target of
    `action_scale` rad/s per unit of action, from rest, after the legs' actions) and the `wheels` section (the joints
    in action order, their SDK indices, and the velocity drive's gains)."""

    joint_names: tuple
    action_scale: float
    stiffness: float
    damping: float
    sdk_ids: tuple | None = None

    @classmethod
    def take(cls, data: dict, source: str) -> "WheelContract | None":
        """Remove the wheels' action term and section from a deploy.yaml's `data` and return them; None, with `data`
        untouched, for a legged policy. Raises ExportedPolicyError for a half-written or unusual wheel contract."""
        actions = data.get("actions") or {}
        if WHEEL_ACTION_TERM not in actions and "wheels" not in data:
            return None
        term, section = actions.pop(WHEEL_ACTION_TERM, None), data.pop("wheels", None)
        if term is None or section is None:
            raise ExportedPolicyError(f"{source}: a wheeled contract needs both actions.{WHEEL_ACTION_TERM} and "
                                      f"wheels; it has only {'the wheels section' if term is None else 'the action'}")
        names = tuple(section.get("joint_names") or ())
        problems = []
        if list(term.get("joint_names") or ()) != list(names):
            problems.append(f"actions.{WHEEL_ACTION_TERM} drives {term.get('joint_names')}, the wheels section "
                            f"names {list(names)}")
        if names != WHEEL_ORDER:
            problems.append(f"the wheels are {list(names)}, not the Go2-W's {list(WHEEL_ORDER)}")
        if section.get("control") != "velocity":
            problems.append(f"the wheels are driven by {section.get('control')!r}; the sim drives them by velocity")
        if term.get("clip") is not None:
            problems.append(f"actions.{WHEEL_ACTION_TERM} clips its targets; the sim does not")
        if float(term.get("offset") or 0.0) != 0.0:
            problems.append(f"actions.{WHEEL_ACTION_TERM} has an offset; the sim's wheels start from rest")
        values = {}
        for key, where in (("scale", term), ("stiffness", section), ("damping", section)):
            v = where.get(key)
            v = [float(x) for x in v] if isinstance(v, (list, tuple)) else ([] if v is None else [float(v)])
            if len(set(v)) != 1:
                problems.append(f"the wheels' {key} is {v}; the sim gives every wheel one value")
            values[key] = v[0] if v else float("nan")
        if problems:
            raise ExportedPolicyError(f"{source} cannot drive the Go2-W's wheels here:\n  " + "\n  ".join(problems))
        sdk = section.get("joint_ids_map")
        return cls(joint_names=names, action_scale=values["scale"], stiffness=values["stiffness"],
                   damping=values["damping"], sdk_ids=tuple(int(i) for i in sdk) if sdk is not None else None)


def _load_contract(directory: Path) -> tuple[DeployContract, WheelContract | None]:
    """The deploy contract, read by `deploy_contract.py` as it reads any, after the wheels (if any) are taken off."""
    for path in DeployContract.candidate_paths(str(directory)):
        if os.path.isfile(path):
            try:
                data = yaml.safe_load(Path(path).read_text())
            except (OSError, yaml.YAMLError) as exc:
                raise DeployContractError(f"{path}: could not be read ({exc})") from exc
            if not isinstance(data, dict):
                raise DeployContractError(f"{path}: expected a YAML mapping, got {type(data).__name__}")
            wheels = WheelContract.take(data, path)
            return DeployContract(data, source=path), wheels
    return DeployContract.load(str(directory)), None     # raises, listing where it looked


@dataclass
class Bundle:
    """An exported policy on disk, checked against what the sim can build. `load` raises ExportedPolicyError with
    every reason it cannot be run."""

    directory: Path
    contract: DeployContract
    scanner: ScannerGeometry | None = None       # None when the policy has no height scan
    scan_offset: float = 0.5
    mask: CameraMask | None = None
    wheels: WheelContract | None = None          # a wheeled (Go2-W) policy's wheels

    @property
    def weights(self) -> Path:
        return self.directory / "policy.pt"

    @property
    def robot(self) -> str:
        """The robot the policy drives: "go2w" with wheels, "go2" without."""
        return "go2w" if self.wheels is not None else "go2"

    @property
    def num_actions(self) -> int:
        return len(ISAAC_LAB_LEG_ORDER) + (len(self.wheels.joint_names) if self.wheels is not None else 0)

    @classmethod
    def load(cls, directory) -> "Bundle":
        directory = Path(directory)
        if not (directory / "policy.pt").is_file():
            raise ExportedPolicyError(f"{directory}: no policy.pt (the TorchScript export) here")
        try:
            contract, wheels = _load_contract(directory)
        except DeployContractError as exc:
            raise ExportedPolicyError(str(exc)) from exc
        problems = contract_problems(contract, wheels)
        bundle = cls(directory=directory, contract=contract, wheels=wheels)
        if contract.uses_height_scan:
            try:
                bundle._read_scan(directory / "params" / "env.yaml")
            except ExportedPolicyError as exc:
                problems.append(str(exc))
        if problems:
            raise ExportedPolicyError(f"{directory} cannot be run here:\n  " + "\n  ".join(problems))
        return bundle

    def _read_scan(self, path: Path) -> None:
        if not path.is_file():
            raise ExportedPolicyError(f"{path} is missing: it records where the height scanner was and how it read")
        env = yaml.load(path.read_text(), Loader=_EnvYamlLoader)
        try:
            scanner = env["scene"]["height_scanner"]
            pattern = scanner["pattern_cfg"]
            term = env["observations"]["policy"]["height_scan"]
        except (KeyError, TypeError) as exc:
            raise ExportedPolicyError(f"{path}: no height scanner or height_scan term ({exc})") from exc
        if not str(pattern.get("func", "")).endswith(":grid_pattern") or tuple(pattern["direction"]) != (0.0, 0.0, -1.0):
            raise ExportedPolicyError(f"{path}: the height scanner is not a downward grid ({pattern.get('func')})")
        self.scanner = ScannerGeometry(
            offset=tuple(float(v) for v in scanner["offset"]["pos"]),
            resolution=float(pattern["resolution"]),
            size=tuple(float(v) for v in pattern["size"]),
            ordering=str(pattern.get("ordering", "xy")),
            ray_alignment=str(scanner.get("ray_alignment") or "yaw"),
        )
        width = self.contract.obs_widths["height_scan"]
        if self.scanner.num_rays != width:
            raise ExportedPolicyError(f"{path}: the scanner casts {self.scanner.num_rays} rays but the contract's "
                                      f"height_scan is {width} wide")
        func = str(term.get("func", "")).rsplit(":", 1)[-1]
        self.scan_offset = float((term.get("params") or {}).get("offset", 0.5))
        if func == CAMERA_SCAN:
            self.mask = CameraMask()
        elif func != STOCK_SCAN:
            raise ExportedPolicyError(f"{path}: the height scan was read by {term.get('func')}, which the sim does "
                                      f"not reproduce (it knows {STOCK_SCAN} and {CAMERA_SCAN})")

    def describe(self) -> str:
        c = self.contract
        scaled = {t: s[0] for t, s in c.obs_scales.items() if s is not None}
        line = (f"{self.directory.name}: {c.obs_dim}-wide observation {c.obs_terms}, scales {scaled or 'none'}, "
                f"{c.control_rate:g} Hz, action scale {c.action_scale:g}")
        if c.command_ranges is not None:
            lows, highs = c.command_ranges
            line += f", commands clamped to {lows}..{highs}"
        if self.scanner is not None:
            line += (f"; height scan centred {self.scanner.offset[0]:g} m ahead"
                     + (f", camera mask (unseen cells {c.height_scan_unobserved:g})" if self.mask else ""))
        if self.wheels is not None:
            line += (f"; wheels {list(self.wheels.joint_names)} by velocity, {self.wheels.action_scale:g} rad/s per "
                     f"unit, kd {self.wheels.damping:g}")
        return line


def contract_problems(contract: DeployContract, wheels: WheelContract | None = None) -> list[str]:
    """Why the sim cannot build this contract's observation or apply its actions; empty when it can."""
    problems = []
    # The joint terms' widths: the legs, and a wheeled policy's wheels after them in the speeds and the actions.
    joints = len(ISAAC_LAB_LEG_ORDER) + (len(wheels.joint_names) if wheels is not None else 0)
    for term, want in (("joint_pos_rel", len(ISAAC_LAB_LEG_ORDER)), ("joint_vel_rel", joints), ("last_action", joints)):
        width = contract.obs_widths.get(term)
        if width is not None and width != want * contract.obs_history[term]:
            problems.append(f"observation {term} is {width} wide; the sim builds {want}"
                            + (" (the legs, then the wheels)" if want != len(ISAAC_LAB_LEG_ORDER) else " (the legs)"))
    unknown = [t for t in contract.obs_terms if t not in SIM_TERMS]
    if unknown:
        problems.append(f"observation terms {unknown} are not built by the sim (it builds {', '.join(SIM_TERMS)})")
    stacked = {t: n for t, n in contract.obs_history.items() if n != 1}
    if stacked:
        problems.append(f"observation history {stacked} is not built by the sim (one frame per term only)")
    if len(contract.default_joint_pos) != len(ISAAC_LAB_LEG_ORDER):
        problems.append(f"the default pose has {len(contract.default_joint_pos)} joints, not the Go2's 12")
    if contract.joint_ids_map is not None and sorted(contract.joint_ids_map) != list(range(len(ISAAC_LAB_LEG_ORDER))):
        problems.append(f"joint_ids_map {contract.joint_ids_map} is not a permutation of the 12 legs")
    return problems


def leg_problems(contract: DeployContract, leg_names, default_pos, tol: float = 1e-5) -> list[str]:
    """Check the sim's legs, in the order its leg terms resolved them, against the contract: the Isaac Lab order,
    and the default pose each joint's `joint_pos_rel` is measured from and each action is added to."""
    problems = []
    if tuple(leg_names) != ISAAC_LAB_LEG_ORDER:
        problems.append(f"the sim's leg terms resolve to {list(leg_names)}, not the Isaac Lab order "
                        f"{list(ISAAC_LAB_LEG_ORDER)} the policy was trained in")
    wrong = [f"{n} {float(got):+.3f} (trained {want:+.3f})"
             for n, got, want in zip(leg_names, default_pos, contract.default_joint_pos) if abs(float(got) - want) > tol]
    if wrong:
        problems.append("default leg pose differs: " + ", ".join(wrong))
    return problems


def resolve(path: str, sim_dir) -> Path:
    """`--policy`'s directory: absolute, or relative to the sim directory (`run_sim.sh` runs from there)."""
    p = Path(path).expanduser()
    if not p.is_absolute():
        p = Path(sim_dir) / p
    if not p.is_dir():
        raise ExportedPolicyError(f"--policy {path}: no such directory ({p})")
    return p.resolve()


class ExportedPolicy:
    """The TorchScript policy, called as the sim calls rsl_rl's: observations in (the wrapper's TensorDict or a
    tensor), raw actions out. The export is the policy mean, deterministic and unclipped."""

    def __init__(self, bundle: Bundle, device):
        self.bundle = bundle
        self.module = torch.jit.load(str(bundle.weights), map_location=device).eval()

    def __call__(self, obs) -> torch.Tensor:
        return self.module(obs if torch.is_tensor(obs) else obs["policy"])
