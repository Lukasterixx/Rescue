"""The wrist RealSense, as D1Training simulates it: the bench D435i's own calibration, the saved wrist mount, the
stereo range limits and noise, and the topics the robot's realsense container publishes.

The numpy half (camera model, calibration, mount, depth noise) is copied from Lukas's D1Training repository,
commit 5e19028: `demos/cup/pick_demo/camera.py` and `realsense.py`. Keep it identical in substance, so the images
a policy or the pick sees here are the ones it saw there.

* **Intrinsics** come from `assets/calibration/d435i_238222076237_640x480.json`, read off the bench camera through
  librealsense (D1Training F-053: 55.6 deg across and a principal point 14 px off centre, where the datasheet preset
  says 69.4 deg and the image centre). `--camera d435` puts the datasheet preset back, as D1Training's pick
  defaults to.
* **The mount** is `assets/mounts/wrist_mount.json`: the camera's colour optical frame in Link6, as it was aligned
  by eye against the CAD in D1Training's console. Not a hand-eye calibration.
* **Depth** is the renderer's perfect depth, zeroed outside the camera's range (as librealsense marks invalid
  pixels) and given Intel's stereo RMS noise inside it, sigma_z = z^2 * 0.08 / (f_depth * baseline). Edge flying
  pixels, holes on dark or shiny surfaces and the projector pattern are not modelled.
* **The rendered eye is not quite where the mount puts it** (D1Training F-052: ~11 mm, constant in Link6, cause
  unexplained). The sim keeps that error rather than hiding it, as D1Training does, because a policy trained there
  saw it too.

On ROS the camera looks like the realsense container: `/camera/color/image_raw` (rgb8) and
`/camera/depth/image_rect_raw`, each with `camera_info`. Unlike the container, depth arrives already on the colour
pixels (both in `camera_color_optical_frame`, same size), as `rs.align` would give it and as D1Training's pick
consumes it, and in float metres (32FC1, 0 = invalid) rather than 16-bit millimetres. The cup pick reads either.
"""
from __future__ import annotations

import array
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

ASSETS = Path(__file__).resolve().parent / "assets"
DEFAULT_CALIBRATION = ASSETS / "calibration" / "d435i_238222076237_640x480.json"
DEFAULT_MOUNT = ASSETS / "mounts" / "wrist_mount.json"
MOUNT_SCHEMA = "d1training.wrist_mount/1"

# Intel's stereo matcher searches 126 disparities; the nearest depth it can report is where that runs out.
DISPARITY_SEARCH = 126
DEFAULT_MAX_DEPTH_M = 3.0         # datasheet ranging limit for a D435
DEFAULT_SUBPIXEL_RMS = 0.08       # Intel's stereo RMS subpixel error
# D1Training camera_body.NEAR_CLIP_PAST_HOUSING_M: nothing nearer the eye than this is drawn, which keeps the
# camera from seeing its own case (4.3 mm ahead of the optical centre, the eye up to 10.7 mm off, F-052).
NEAR_CLIP_M = 0.020
FAR_CLIP_M = 10.0

COLOUR_FRAME = "camera_color_optical_frame"


@dataclass(frozen=True)
class CameraModel:
    """Colour intrinsics (depth is aligned to colour, as `rs.align` does) and the depth limits."""

    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    min_depth_m: float
    max_depth_m: float
    baseline_m: float
    depth_fx: float
    subpixel_rms: float
    source: str

    @property
    def intrinsic_matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def depth_noise_std_m(self, z):
        """Intel's stereo RMS error model, sigma_z = z^2 * subpixel / (f * baseline). Best case: a
        textured, well-lit, fronto-parallel surface. Real surfaces are noisier."""
        z = np.asarray(z, dtype=float)
        return z * z * self.subpixel_rms / (self.depth_fx * self.baseline_m)


def _colour_fx(width_native: int, hfov_deg: float, scale: float) -> float:
    return (width_native / 2.0) / math.tan(math.radians(hfov_deg) / 2.0) * scale


def _min_z(depth_width: int, depth_hfov_deg: float, baseline_m: float) -> tuple[float, float]:
    f = (depth_width / 2.0) / math.tan(math.radians(depth_hfov_deg) / 2.0)
    return f, f * baseline_m / DISPARITY_SEARCH


def _preset(name, native_w, native_h, colour_hfov, depth_hfov, depth_width, baseline, max_depth, source):
    scale = 480.0 / native_h
    fx = _colour_fx(native_w, colour_hfov, scale)
    depth_fx, min_z = _min_z(depth_width, depth_hfov, baseline)
    return CameraModel(name, 640, 480, fx, fx, 320.0, 240.0, round(min_z, 3), max_depth, baseline, depth_fx,
                       DEFAULT_SUBPIXEL_RMS, source)


# D1Training camera.CAMERAS: datasheet-derived presets, not calibrations.
PRESETS = {
    "d435": _preset("d435", 1920, 1080, 69.4, 87.0, 848, 0.050, 3.0,
                    "derived from the Intel D400 datasheet FOV and baseline; not a calibration"),
    "d455": _preset("d455", 1280, 800, 90.0, 87.0, 848, 0.095, 4.0,
                    "derived from the Intel D400 datasheet FOV and baseline; not a calibration"),
    "d405": CameraModel("d405", 640, 480, _colour_fx(1280, 87.0, 480.0 / 720.0), _colour_fx(1280, 87.0, 480.0 / 720.0),
                        320.0, 240.0, 0.07, 0.50, 0.018, _min_z(848, 87.0, 0.018)[0], DEFAULT_SUBPIXEL_RMS,
                        "Intel D405 datasheet FOV, baseline and 7-50 cm rated range; not a calibration"),
}


def load_calibration(path) -> dict:
    return json.loads(Path(path).read_text())


def camera_model(calibration: dict, max_depth_m: float = DEFAULT_MAX_DEPTH_M,
                 subpixel_rms: float = DEFAULT_SUBPIXEL_RMS) -> CameraModel:
    """A `CameraModel` describing the camera the calibration came from (D1Training `realsense.camera_model`)."""
    colour, depth = calibration["colour_intrinsics"], calibration["depth_intrinsics"]
    baseline = float(calibration["stereo_baseline_m"])
    min_depth = depth["fx"] * baseline / DISPARITY_SEARCH
    device = calibration["device"]
    return CameraModel(
        name=f"{device.get('name', 'realsense')} #{device.get('serial', '?')}",
        width=int(colour["width"]), height=int(colour["height"]),
        fx=float(colour["fx"]), fy=float(colour["fy"]), cx=float(colour["cx"]), cy=float(colour["cy"]),
        min_depth_m=round(float(min_depth), 4), max_depth_m=float(max_depth_m),
        baseline_m=baseline, depth_fx=float(depth["fx"]), subpixel_rms=float(subpixel_rms),
        source=(f"measured: librealsense on {device.get('name')} #{device.get('serial')} "
                f"at {colour['width']}x{colour['height']}, captured {calibration.get('captured_utc')}; "
                f"min_depth is the {DISPARITY_SEARCH}-disparity limit and max_depth the datasheet's"),
    )


def resolve_camera(choice: str) -> CameraModel:
    """`choice` is a preset name (d435, d405, d455), a calibration file, or "calibration" for the bench D435i."""
    if choice in PRESETS:
        return PRESETS[choice]
    return camera_model(load_calibration(DEFAULT_CALIBRATION if choice == "calibration" else choice))


def realsense_depth(model: CameraModel, depth_true, rng: np.random.Generator | None = None) -> np.ndarray:
    """What the camera would report for a perfect rendered depth: zero outside its range (as librealsense
    marks invalid pixels), Gaussian noise from `depth_noise_std_m` inside it. Verbatim from D1Training."""
    depth = np.asarray(depth_true, dtype=np.float32).copy()
    valid = np.isfinite(depth) & (depth >= model.min_depth_m) & (depth <= model.max_depth_m)
    if rng is not None:
        noise = rng.standard_normal(depth.shape).astype(np.float32)
        depth = np.where(valid, depth + noise * model.depth_noise_std_m(np.where(valid, depth, 0.0)), 0.0)
    return np.where(valid, depth, 0.0).astype(np.float32)


# ------------------------------------------------------------------------------------------ the mount
def rpy_matrix_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) Ry(pitch) Rx(roll), radians: the URDF convention and the mount file's."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_quat_wxyz(rot) -> tuple[float, float, float, float]:
    rot = np.asarray(rot, dtype=float)
    trace = np.trace(rot)
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w, x, y, z = 0.25 / s, (rot[2, 1] - rot[1, 2]) * s, (rot[0, 2] - rot[2, 0]) * s, (rot[1, 0] - rot[0, 1]) * s
    else:
        i = int(np.argmax(np.diag(rot)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(rot[i, i] - rot[j, j] - rot[k, k] + 1.0, 0.0)) * 2.0
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (rot[j, i] + rot[i, j]) / s
        q[k] = (rot[k, i] + rot[i, k]) / s
        w = (rot[k, j] - rot[j, k]) / s
        x, y, z = q
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in (q if q[0] >= 0.0 else -q))


@dataclass(frozen=True)
class Mount:
    """The camera's colour optical frame in the Link6 frame, as a 4x4, and where that came from."""

    pose: np.ndarray
    source: str

    @property
    def rotation(self) -> np.ndarray:
        return self.pose[:3, :3]

    @property
    def pos_link6(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.pose[:3, 3])

    def quat_wxyz(self) -> tuple[float, float, float, float]:
        return matrix_to_quat_wxyz(self.rotation)


def load_mount(path) -> Mount:
    """A `d1training.wrist_mount/1` file. The 4x4 is the authority; `xyz_m`/`rpy_deg` must agree with it."""
    data = json.loads(Path(path).read_text())
    if data.get("schema") != MOUNT_SCHEMA:
        raise ValueError(f"{path}: schema is {data.get('schema')!r}, expected {MOUNT_SCHEMA!r}")
    pose = np.asarray(data["pose_link6"], dtype=float)
    if pose.shape != (4, 4):
        raise ValueError(f"{path}: pose_link6 is {pose.shape}, expected 4x4")
    rebuilt = np.eye(4)
    rebuilt[:3, :3] = rpy_matrix_zyx(*(math.radians(float(v)) for v in data["rpy_deg"]))
    rebuilt[:3, 3] = [float(v) for v in data["xyz_m"]]
    if not np.allclose(rebuilt, pose, atol=1e-6):
        raise ValueError(f"{path}: xyz_m/rpy_deg and pose_link6 disagree; the file was edited in one place only")
    kind = "measured" if data.get("measured") else "aligned by eye, not measured"
    return Mount(pose, f"{data.get('source', path)} ({kind}: {data.get('method') or 'unstated'})")


# ------------------------------------------------------------------------------------------ on ROS
def rendered_intrinsics(model: CameraModel, rendered_k) -> tuple[tuple[float, float, float, float], str]:
    """The intrinsics the renderer really uses, and a sentence on how they differ from the model's.

    Omniverse renders square pixels about the image centre: Isaac Lab spawns a camera from an intrinsic matrix by
    averaging fx and fy and dropping the principal point's offset. The bench D435i's principal point is 3.0 px right
    of centre and 14.3 px below it (F-053), so its images come out as if taken by a camera with a centred one. The
    published camera_info describes the images, so it carries these, not the calibration's."""
    k = np.asarray(rendered_k, dtype=float).reshape(3, 3)
    fx, fy, cx, cy = float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
    note = (f"rendered fx {fx:.2f} fy {fy:.2f} cx {cx:.2f} cy {cy:.2f}; the model's {model.fx:.2f} {model.fy:.2f} "
            f"{model.cx:.2f} {model.cy:.2f} (Omniverse renders square pixels about the centre)")
    return (fx, fy, cx, cy), note


class WristCameraPublisher:
    """Colour, aligned depth and their camera_info, on the realsense container's topic names. `intrinsics` (fx, fy,
    cx, cy) are what the renderer used (`rendered_intrinsics`); the model supplies the depth range and noise."""

    def __init__(self, node, model: CameraModel, rng: np.random.Generator | None, intrinsics=None,
                 colour_topic="/camera/color/image_raw", depth_topic="/camera/depth/image_rect_raw"):
        from sensor_msgs.msg import CameraInfo, Image

        self._Image, self._CameraInfo = Image, CameraInfo
        self.model, self.rng = model, rng
        self.intrinsics = tuple(intrinsics) if intrinsics is not None else (model.fx, model.fy, model.cx, model.cy)
        self._colour = node.create_publisher(Image, colour_topic, 5)
        self._depth = node.create_publisher(Image, depth_topic, 5)
        self._colour_info = node.create_publisher(CameraInfo, colour_topic.rsplit("/", 1)[0] + "/camera_info", 5)
        self._depth_info = node.create_publisher(CameraInfo, depth_topic.rsplit("/", 1)[0] + "/camera_info", 5)
        self.frames = 0

    def _info(self, stamp) -> "CameraInfo":
        fx, fy, cx, cy = self.intrinsics
        info = self._CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, COLOUR_FRAME
        info.width, info.height = self.model.width, self.model.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish(self, rgb: np.ndarray, depth_true: np.ndarray, stamp) -> np.ndarray:
        """Publish one frame. `rgb` is (H, W, 3) uint8, `depth_true` (H, W) metres along the optical axis. Returns
        the depth as published."""
        depth = realsense_depth(self.model, depth_true, self.rng)
        height, width = depth.shape
        colour = self._Image()
        colour.header.stamp, colour.header.frame_id = stamp, COLOUR_FRAME
        colour.height, colour.width, colour.encoding, colour.step = height, width, "rgb8", 3 * width
        # An array('B') is taken as it is. Anything else goes through rclpy's per-element check, which on a 640x480
        # frame costs a fifth of a second.
        colour.data = array.array("B", np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())
        aligned = self._Image()
        aligned.header.stamp, aligned.header.frame_id = stamp, COLOUR_FRAME
        aligned.height, aligned.width, aligned.encoding, aligned.step = height, width, "32FC1", 4 * width
        aligned.data = array.array("B", np.ascontiguousarray(depth, dtype=np.float32).tobytes())
        self._colour.publish(colour)
        self._depth.publish(aligned)
        info = self._info(stamp)
        self._colour_info.publish(info)
        self._depth_info.publish(info)
        self.frames += 1
        return depth
