# Copyright (c) 2024, RoboVerse community
# ... (Header unchanged) ...

import asyncio
import time
import numpy as np

from arm_mount import ee_prim_path
import cv2                 # OpenCV for video recording
import os, json, math
import importlib

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rcl_interfaces.msg import SetParametersResult

# --- Message Imports ---
from sensor_msgs.msg import JointState, PointCloud2, PointField, Imu
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import Header, String, Empty
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs_py import point_cloud2

# --- Omniverse / Pixar Imports ---
from pxr import Gf, Sdf, UsdGeom
import omni.usd
import omni.timeline
import carb
import omni.replicator.core as rep
from isaaclab.sensors import CameraCfg, Camera
from isaacsim.sensors.rtx import LidarRtx
import isaaclab.sim as sim_utils
from scipy.spatial.transform import Rotation
from PIL import Image
import datetime
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time as RosTime

DISABLE_RING_AND_TIME = True    # fast mode
LIVOX_OFFSET_BASE = np.array([0.3, 0.0, 0.25], dtype=float)
NATIVE_IMU_TOPIC = "/livox/imu"
NATIVE_IMU_FRAME_ID = "livox_frame"
NATIVE_IMU_RATE_HZ = 100.0

# Global references to prevent sensors from being garbage collected
_lidars_keep_alive = []
_cameras_keep_alive = []
_lidar_render_products = []
_lidar_debug_enabled = False
_lidar_debug_writer = None

def get_isaac_sim_time_msg(env):
    """Return Isaac/IsaacLab simulation time as a ROS Time message."""
    sim_time = None

    # Isaac Lab usually exposes sim time here.
    try:
        sim_time = float(env.unwrapped.sim.current_time)
    except Exception:
        pass

    # Fallback: step counter * physics/render step.
    if sim_time is None:
        try:
            sim_time = float(env.unwrapped.common_step_counter) * float(env.unwrapped.step_dt)
        except Exception:
            sim_time = 0.0

    sec = int(sim_time)
    nanosec = int((sim_time - sec) * 1e9)

    msg = RosTime()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg

def get_storage_path(batch_id):
    """
    Returns: .../share/go2_control_cpp/photos/<batch_id>
    batch_id is now a string (e.g., "1", "1.1")
    """
    try:
        pkg_path = get_package_share_directory('go2_control_cpp')
        base_path = os.path.join(pkg_path, 'photos')
    except Exception as e:
        print(f"Warning: Could not find package 'go2_control_cpp'. Error: {e}")
        base_path = os.path.expanduser("~/P2Dingo/photos")

    full_path = os.path.join(base_path, str(batch_id))
    return full_path

def save_images_to_disk(annotator_list, batch_id):
    batch_path = get_storage_path(batch_id)
    os.makedirs(batch_path, exist_ok=True)
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    
    for i, anno in enumerate(annotator_list):
        try:
            data = anno.get_data()
            if data is None:
                continue
            if data.shape[-1] == 4:
                data = data[..., :3]
            
            im = Image.fromarray(data.astype(np.uint8))
            # Include batch_id in filename for clarity
            filename = f"{batch_path}/photo_{batch_id}_cam{i}_{timestamp}.png"
            im.save(filename)
            print(f"Saved photo to: {filename}")
            
        except Exception as e:
            print(f"Failed to save image for cam {i}: {e}")

# ... (Helper functions load_lidar_json, compute_ring..., update_meshes..., add_rtx_lidar... UNCHANGED) ...
def load_lidar_json(json_path: str):
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Lidar JSON not found: {json_path}")
    with open(json_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    prof = cfg["profile"]
    emit = prof.get("emitters", {})
    horizon_scan = 1024
    report_hz = float(prof.get("reportRateBaseHz", 10240))
    spin_hz = max(report_hz / float(horizon_scan), 1e-6)
    scan_period = 1.0 / spin_hz
    elev_deg = np.array(emit.get("elevationDeg", []), dtype=np.float32)
    fire_time_s = np.array(emit.get("fireTimeNs", []), dtype=np.float32) * 1e-9
    n_scan = int(prof.get("numberOfEmitters", elev_deg.size if elev_deg.size else 0))

    return {
        "scan_period": scan_period,
        "elev_deg": elev_deg,
        "fire_time_s": fire_time_s,
        "n_scan": n_scan,
        "near_m": float(prof.get("nearRangeM", 0.5)),
        "far_m": float(prof.get("farRangeM", 75.0)),
        "horizon_scan": horizon_scan,
    }

def compute_ring_from_elevation(points_xyz: np.ndarray, elev_deg_list: np.ndarray) -> np.ndarray:
    if elev_deg_list.size == 0:
        x, y, z = points_xyz[:,0], points_xyz[:,1], points_xyz[:,2]
        elev = np.degrees(np.arctan2(z, np.sqrt(x*x + y*y) + 1e-9))
        n = 64
        bins = np.linspace(elev.min(), elev.max(), n, endpoint=True)
        return np.clip(np.digitize(elev, bins) - 1, 0, n-1).astype(np.uint16)

    x, y, z = points_xyz[:,0], points_xyz[:,1], points_xyz[:,2]
    horiz = np.sqrt(x*x + y*y) + 1e-9
    elev = np.degrees(np.arctan2(z, horiz))
    diff = np.abs(elev[:, None] - elev_deg_list[None, :])
    rings = diff.argmin(axis=1).astype(np.uint16)
    return rings

def compute_time_from_azimuth(points_xyz: np.ndarray, scan_period: float,
                              ring_idx: np.ndarray | None = None,
                              ring_fine_offset_s: np.ndarray | None = None) -> np.ndarray:
    x, y = points_xyz[:,0], points_xyz[:,1]
    az = np.arctan2(y, x)
    az = np.mod(az + 2*np.pi, 2*np.pi)
    t = (az / (2*np.pi)) * scan_period
    if ring_idx is not None and ring_fine_offset_s is not None and ring_fine_offset_s.size:
        t = (t + ring_fine_offset_s[ring_idx]) % scan_period
    return t.astype(np.float32)

def update_meshes_for_cloud2(position_array, origin, rot):
    pts = position_array.copy()
    # Apply LIVOX_OFFSET_BASE using numpy broadcasting
    pts += LIVOX_OFFSET_BASE
    return pts

def toggle_lidar_debug_draw():
    global _lidar_debug_enabled, _lidar_render_products, _lidar_debug_writer
    if not _lidar_render_products:
        return
    _lidar_debug_enabled = not _lidar_debug_enabled
    
    if _lidar_debug_writer is None:
        _lidar_debug_writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
        
    if _lidar_debug_enabled:
        _lidar_debug_writer.attach(_lidar_render_products)
        print("[LIDAR] Debug drawing ENABLED")
    else:
        _lidar_debug_writer.detach()
        print("[LIDAR] Debug drawing DISABLED")

def create_rtx_lidar_ros2_graph(robot_num, lidar_sensor, topic_name="/glim_rosnode/points"):
    import omni.graph.core as og
    keys = og.Controller.Keys
    graph_path = f"/ROS_RTX_LIDAR_{robot_num}"
    render_product_path = lidar_sensor.get_render_product_path()

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RtxLidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("RtxLidarHelper.inputs:enabled", True),
                ("RtxLidarHelper.inputs:renderProductPath", render_product_path),
                ("RtxLidarHelper.inputs:topicName", topic_name),
                ("RtxLidarHelper.inputs:frameId", "livox_frame"),
                ("RtxLidarHelper.inputs:type", "point_cloud"),
                ("RtxLidarHelper.inputs:fullScan", True),
                ("RtxLidarHelper.inputs:frameSkipCount", 0),
                ("RtxLidarHelper.inputs:queueSize", 10),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RtxLidarHelper.inputs:execIn"),
                ("ROS2Context.outputs:context", "RtxLidarHelper.inputs:context"),
            ],
        },
    )

def _set_attr_if_present(prim, names, value):
    for attr_name in names:
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            attr.Set(value)
            print(f"[IMU][OVERRIDE] {attr_name} = {value}")
            return True
    return False


def _set_graph_target(graph_path, node_name, input_name, target_path):
    import omni.graph.core as og

    attr = og.Controller.attribute(f"{graph_path}/{node_name}.inputs:{input_name}")
    if attr is None:
        raise RuntimeError(f"Missing OmniGraph target input: {node_name}.inputs:{input_name}")

    try:
        attr.set([Sdf.Path(target_path)])
    except Exception:
        attr.set([target_path])


def _update_isaac_app_once():
    try:
        kit_app = importlib.import_module("omni.kit.app")
        kit_app.get_app().update()
    except Exception:
        pass


def create_native_imu_ros2_graph(robot_num, imu_prim_path, topic_name=NATIVE_IMU_TOPIC):
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph_path = f"/ROS_NATIVE_IMU_{robot_num}"

    print(f"[IMU][ROS2] publishing native IMU on {topic_name} from {imu_prim_path}")

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("ReadIMU.inputs:readGravity", True),
                ("ReadIMU.inputs:useLatestData", False),
                ("PublishIMU.inputs:frameId", NATIVE_IMU_FRAME_ID),
                ("PublishIMU.inputs:topicName", topic_name),
                ("PublishIMU.inputs:nodeNamespace", ""),
                ("PublishIMU.inputs:queueSize", 10),
                ("PublishIMU.inputs:publishOrientation", True),
                ("PublishIMU.inputs:publishLinearAcceleration", True),
                ("PublishIMU.inputs:publishAngularVelocity", True),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishIMU.inputs:context"),
                ("ReadIMU.outputs:sensorTime", "PublishIMU.inputs:timeStamp"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ],
        },
    )

    _set_graph_target(graph_path, "ReadIMU", "imuPrim", imu_prim_path)


def add_native_imu(robot_num, parent_path, translation, orientation=(1.0, 0.0, 0.0, 0.0)):
    import omni.kit.commands

    imu_prim_path = f"{parent_path}/livox_imu"
    stage = omni.usd.get_context().get_stage()
    existing_prim = stage.GetPrimAtPath(imu_prim_path)

    if not existing_prim or not existing_prim.IsValid():
        success = False
        imu_prim = None
        command_kwargs = {
            "path": "livox_imu",
            "parent": parent_path,
            "linear_acceleration_filter_size": 1,
            "angular_velocity_filter_size": 1,
            "orientation_filter_size": 1,
            "translation": Gf.Vec3d(*translation),
            "orientation": Gf.Quatd(*orientation),
        }

        command_name = "IsaacSensorCreateImuSensor"
        try:
            success, imu_prim = omni.kit.commands.execute(command_name, **command_kwargs)
            if success:
                print(f"[IMU][CREATE] {command_name} created {imu_prim_path}")
        except Exception as e:
            print(f"[IMU][CREATE][FAILED] {command_name}: {e}")

        if not success:
            raise RuntimeError(f"Failed to create native IMU sensor at {imu_prim_path}")
    else:
        imu_prim = existing_prim
        print(f"[IMU][CREATE] Reusing existing IMU prim: {imu_prim_path}")

    stage = omni.usd.get_context().get_stage()
    imu_prim = stage.GetPrimAtPath(imu_prim_path)
    if not imu_prim or not imu_prim.IsValid():
        raise RuntimeError(f"Invalid IMU prim after creation: {imu_prim_path}")

    sensor_period = 1.0 / NATIVE_IMU_RATE_HZ
    _set_attr_if_present(
        imu_prim,
        (
            "sensorPeriod",
            "imuSensor:sensorPeriod",
            "physxSensor:sensorPeriod",
            "omni:sensor:Core:sensorPeriod",
        ),
        sensor_period,
    )
    _set_attr_if_present(imu_prim, ("enabled", "imuSensor:enabled", "physxSensor:enabled"), True)

    create_native_imu_ros2_graph(robot_num, imu_prim_path, topic_name=NATIVE_IMU_TOPIC)
    return imu_prim_path

def add_rtx_lidar(num_envs, robot_type, debug=False):
    annotator_lst = []
    global _lidars_keep_alive
    global _lidar_render_products
    global _lidar_debug_enabled
    global _lidar_debug_writer

    _lidar_debug_enabled = debug
    _lidar_render_products.clear()

    settings = carb.settings.get_settings()
    settings.set_bool("/app/sensors/nv/lidar/outputBufferOnGPU", True)
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        _update_isaac_app_once()
    config_name = "Example_Rotary"

    for i in range(num_envs):
        if robot_type == "g1":
            sensor_parent_path = f"/World/envs/env_{i}/Robot/head_link"
            lidar_prim_path = f"{sensor_parent_path}/lidar_sensor"
            lidar_translation = (0.0, 0.0, 0.0)
        else:
            sensor_parent_path = f"/World/envs/env_{i}/Robot/base"
            lidar_prim_path = f"{sensor_parent_path}/lidar_sensor"
            lidar_translation = tuple(LIVOX_OFFSET_BASE)

        lidar_sensor = LidarRtx(
            prim_path=lidar_prim_path,
            translation=lidar_translation,
            orientation=(1.0, 0.0, 0.0, 0.0),
            config_file_name=config_name,
        )
        lidar_sensor.initialize()

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(lidar_sensor.prim_path)
        
        if not prim or not prim.IsValid():
            annotator_lst.append(None)
            _lidars_keep_alive.append(lidar_sensor)
            continue

        # Rate/range overrides
        safe_overrides = {
            # Increased 5x to maintain point density
            "omni:sensor:Core:reportRateBaseHz": 4000,
            # Increased from 10Hz to 50Hz to eliminate motion distortion during fast turns
            "omni:sensor:Core:scanRateBaseHz": 50,
            "omni:sensor:Core:nearRangeM": 0.6,
            "omni:sensor:Core:farRangeM": 30.0,
            "omni:sensor:Core:numberOfEmitters": 128,
            "omni:sensor:Core:numberOfChannels": 128,
        }

        for attr_name, value in safe_overrides.items():
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid():
                attr.Set(value)

        # Livox-ish blended vertical layout
        elev_attr_name = "omni:sensor:Core:emitterState:s001:elevationDeg"
        elev_attr = prim.GetAttribute(elev_attr_name)

        if elev_attr and elev_attr.IsValid():
            vertical_channels_per_group = 32
            num_azimuth_groups = 4
            vertical_min_deg = -15.0
            vertical_max_deg = 45.0

            base_stack = np.linspace(
                vertical_min_deg,
                vertical_max_deg,
                vertical_channels_per_group,
            ).astype(np.float32)

            rng = np.random.default_rng(seed=42)
            all_elev = []
            jitter_std_deg = 0.45

            for group_idx in range(num_azimuth_groups):
                group_jitter = rng.normal(
                    loc=0.0,
                    scale=jitter_std_deg,
                    size=vertical_channels_per_group,
                ).astype(np.float32)

                group_stack = base_stack + group_jitter
                group_stack = np.clip(group_stack, vertical_min_deg, vertical_max_deg)
                group_stack = np.sort(group_stack)
                all_elev.extend(group_stack.tolist())

            elev_attr.Set(all_elev)

        create_rtx_lidar_ros2_graph(i, lidar_sensor, topic_name="/glim_rosnode/points")
        add_native_imu(i, sensor_parent_path, lidar_translation)
        
        _lidar_render_products.append(lidar_sensor.get_render_product_path())

        annotator_lst.append(None) # Not strictly needed if using raw ROS bridge
        _lidars_keep_alive.append(lidar_sensor)
        
    if _lidar_debug_enabled and _lidar_render_products:
        if _lidar_debug_writer is None:
            _lidar_debug_writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
        _lidar_debug_writer.attach(_lidar_render_products)

    return annotator_lst

def add_ee_flashlight(num_envs, robot_type):
    """Attach a strong forward-facing flashlight spotlight to the D1 end-effector."""
    from pxr import UsdLux, UsdGeom, Gf
    import omni.usd

    stage = omni.usd.get_context().get_stage()

    for i in range(num_envs):
        if robot_type == "g1":
            parent_path = f"/World/envs/env_{i}/Robot/head_link"
        else:
            parent_path = ee_prim_path(i)

        parent_prim = stage.GetPrimAtPath(parent_path)
        if not parent_prim or not parent_prim.IsValid():
            print(f"[FLASHLIGHT][WARN] Missing parent prim: {parent_path}")
            continue

        light_path = f"{parent_path}/flashlight"

        light = UsdLux.SphereLight.Define(stage, light_path)
        light.GetPrim().SetTypeName("SphereLight")

        # Stronger bulb-style emitter
        light.CreateIntensityAttr(500000.0)
        light.CreateRadiusAttr(0.04)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.85))
        light.CreateExposureAttr(2.0)

        xform = UsdGeom.Xformable(light.GetPrim())
        xform.ClearXformOpOrder()
        # Out along the gripper's approach axis (Link6 +Z), which is where the
        # camera now looks. This used to sit on +X, lighting the floor under the
        # wrist rather than whatever is in front of the jaws.
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.15))

        print(f"[FLASHLIGHT] Added stronger end-effector light at {light_path}")

# Where the camera sits on Link6, in Link6's own axes.
#
# +Z is the gripper's approach axis, so 0.05 along it puts the lens out past the
# wrist. -X is "up" out of the back of the hand (Link6 +X is the image-down axis
# and points floor-ward), so -0.04 lifts the camera clear of the jaws -- they
# occupy roughly z 0.07..0.12 on the +Z axis, and sitting on that axis put the
# lens inside the claw.
#
# add_camera() and the arm_camera_link broadcast both read this. They describe
# one physical camera and must not drift apart: the scanner casts its FOV rays
# from arm_camera_link, so if that frame is not where the lens is, the rays start
# somewhere the camera cannot see from.
_CAM_OFFSET_IN_LINK6 = (-0.04, 0.0, 0.05)


def add_camera(num_envs, robot_type):
    annotators = []
    global _cameras_keep_alive

    for i in range(num_envs):
        # Mount the camera to Link6 (end-effector) of the D1 arm
        cameraCfg = CameraCfg(
            prim_path=f"{ee_prim_path(i)}/front_cam",
            update_period=0.1,
            # Standard RealSense resolution
            height=480,
            width=848,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                # RealSense optics to achieve ~87° HFOV
                focal_length=1.93, 
                focus_distance=400.0, 
                horizontal_aperture=3.64,
                # Lower near-clipping plane because arms get close to objects
                clipping_range=(0.01, 1.0e5) 
            ),
            # Look where the gripper points.
            #
            # Link6's +Z is the approach axis -- the direction the jaws close
            # along, and the one D1_EE_REST_ROT is described against. +X is
            # across the wrist, and at the arm's zero pose it points at the
            # floor. The old (0.5, -0.5, 0.5, -0.5) put the optical axis on +X,
            # so the camera stared straight down out of the gripper.
            #
            # A "ros" offset gives the optical frame (X right, Y down, Z fwd) in
            # the parent's axes, so we want Z_opt = Link6 +Z. Y_opt (image down)
            # = Link6 +X, which points floor-ward at rest and so keeps the image
            # upright; X_opt = Y_opt x Z_opt = Link6 -Y. That is a -90 deg turn
            # about Link6's Z.
            offset=CameraCfg.OffsetCfg(
                pos=_CAM_OFFSET_IN_LINK6,
                rot=(0.70710678, 0.0, 0.0, -0.70710678),
                convention="ros",
            ),
        )

        # (If using the g1 robot, you might want to bypass or update this condition)
        if robot_type == "g1":
            cameraCfg.prim_path = f"/World/envs/env_{i}/Robot/head_link/front_cam"
            cameraCfg.offset = CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.5, -0.5, 0.5, -0.5), convention="ros")

        cam = Camera(cameraCfg)
        _cameras_keep_alive.append(cam)

        # Ensure the render product matches the new resolution
        render_prod = rep.create.render_product(cameraCfg.prim_path, (848, 480))
        rgb_anno = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_anno.attach(render_prod)
        annotators.append(rgb_anno)
    return annotators

def _extract_points(bundle):
    for k in ("data", "points", "xyz", "pointData", "point_cloud"):
        if k in bundle:
            pts = np.asarray(bundle[k])
            if pts.ndim == 1:
                if pts.size % 3 != 0:
                    raise ValueError(f"LiDAR points buffer size {pts.size} not divisible by 3")
                pts = pts.reshape(-1, 3)
            return pts.astype(np.float32)
    raise KeyError(f"No XYZ key found in annotator bundle: {list(bundle.keys())}")

def _extract_optional(bundle, *candidates):
    for k in candidates:
        if k in bundle:
            arr = np.asarray(bundle[k])
            return arr
    return None


def pub_robo_data_ros2(robot_type, num_envs, base_node, env, annotator_lst, next_deadline, camera_lst=None, ee_pos=None, ee_quat=None):
    # --- Check for photo request (triggered by string ID) ---
    if base_node.photo_requested and camera_lst is not None:
        save_images_to_disk(camera_lst, base_node.photo_batch_id)
        base_node.photo_requested = False

    # --- Video recording (Target 30 FPS) into string-named folder ---
    if base_node.is_recording and camera_lst is not None and len(camera_lst) > 0:
        now_mono = time.monotonic()
        if now_mono - base_node.last_video_frame_time >= (1.0 / 30.0):
            try:
                anno_data = camera_lst[0].get_data()
                if anno_data is not None:
                    rgb_data = anno_data[..., :3] if anno_data.shape[-1] == 4 else anno_data
                    bgr_frame = rgb_data[..., ::-1].copy()
                    
                    if base_node.video_writer is None:
                        h, w = bgr_frame.shape[:2]
                        batch_path = get_storage_path(base_node.video_rec_batch_id)
                        os.makedirs(batch_path, exist_ok=True)
                        
                        video_path = os.path.join(batch_path, f"video_{base_node.video_rec_batch_id}.mp4")
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        base_node.video_writer = cv2.VideoWriter(video_path, fourcc, 30.0, (w, h))
                        base_node.get_logger().info(f"Started video in: {batch_path}")

                    base_node.video_writer.write(bgr_frame.astype(np.uint8))
                    base_node.last_video_frame_time = now_mono
            except Exception as e:
                base_node.get_logger().error(f"Video record error: {e}")

    # --- NEW: Update Camera Orientation in Sim (Existing logic) ---
    if base_node.camera_target_pose is not None:
        stage = omni.usd.get_context().get_stage()
        q_ros = base_node.camera_target_pose.orientation
        target_rot = Gf.Quatd(q_ros.w, q_ros.x, q_ros.y, q_ros.z)
        smoothing_factor = 0.15 
        base_node.cam_current_rot = Gf.Slerp(smoothing_factor, base_node.cam_current_rot, target_rot)
        correction_rot = Gf.Quatd(0.5, 0.5, -0.5, -0.5)        
        final_rot = base_node.cam_current_rot * correction_rot

        for i in range(num_envs):
            # Updated to target the new arm-mounted camera path
            cam_prim_path = f"{ee_prim_path(i)}/front_cam"
            prim = stage.GetPrimAtPath(cam_prim_path)
            if prim.IsValid():
                xform = UsdGeom.Xformable(prim)
                rotate_op = None
                for op in xform.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                        rotate_op = op
                        break
                if not rotate_op:
                    rotate_op = xform.AddOrientOp()
                rotate_op.Set(final_rot)

    # --- Standard Pub Logic ---
    # Use Isaac simulation time, not wall-clock time.
    time_now_for_clock = get_isaac_sim_time_msg(env)

    clock_msg = Clock()
    clock_msg.clock = time_now_for_clock
    base_node.clock_pub.publish(clock_msg)

    for i in range(num_envs):
        base_node.publish_joints(
            env.unwrapped.scene["robot"].data.joint_names,
            env.unwrapped.scene["robot"].data.joint_pos[i],
            i,
            time_now_for_clock
        )
        base_node.publish_odom(
            env.unwrapped.scene["robot"].data.root_state_w[i, :3],
            env.unwrapped.scene["robot"].data.root_state_w[i, 3:7],
            i,
            time_now_for_clock
        )

    # --- NEW: Broadcast Arm End-Effector TF ---
    if ee_pos is not None and ee_quat is not None:
        ee_trans = TransformStamped()
        ee_trans.header.stamp = time_now_for_clock
        ee_trans.header.frame_id = "odom"
        ee_trans.child_frame_id = "arm_camera_link"

        # Isaac provides arrays: pos [x,y,z], quat [w,x,y,z] in World Frame
        px, py, pz = float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])
        qw, qx, qy, qz = float(ee_quat[0]), float(ee_quat[1]), float(ee_quat[2]), float(ee_quat[3])

        # Link6 is not a camera frame, and consumers treat this one as if it
        # were: the wall scanner reads this frame's yaw and casts its FOV rays
        # down +X, from this frame's origin. So hand them a proper ROS camera
        # body frame -- +X along the view direction, +Z up, sitting where the
        # lens actually is -- rather than Link6 raw, whose +X points across the
        # wrist (at the floor, when the arm is at zero).
        #
        # The camera looks down Link6's +Z, so: X_body = Link6 +Z, Z_body =
        # Link6 -X, Y_body = Z_body x X_body = Link6 +Y. That is a -90 deg turn
        # about Link6's Y.
        _LINK6_TO_CAM = Rotation.from_quat([0.0, -0.70710678, 0.0, 0.70710678])  # x,y,z,w
        _link6_rot = Rotation.from_quat([qx, qy, qz, qw])

        _lens = np.array([px, py, pz]) + _link6_rot.apply(_CAM_OFFSET_IN_LINK6)
        px, py, pz = float(_lens[0]), float(_lens[1]), float(_lens[2])

        _cam_rot = _link6_rot * _LINK6_TO_CAM
        qx, qy, qz, qw = _cam_rot.as_quat()

        # We must apply the same odom origin and nudge as the base
        if 0 in base_node.odom_origins:
            origin = base_node.odom_origins[0]
            ee_rot = Rotation.from_quat([qx, qy, qz, qw])
            R_rel = origin["rot"].inv() * ee_rot
            
            P_diff = np.array([px, py, pz]) - origin["pos"]
            P_rel = origin["rot"].inv().apply(P_diff)
            
            px, py, pz = P_rel[0], P_rel[1], P_rel[2]
            
            final_quat = R_rel.as_quat() # [x, y, z, w]
            qx, qy, qz, qw = final_quat[0], final_quat[1], final_quat[2], final_quat[3]

        if hasattr(base_node, 'accumulated_nudge'):
            px += base_node.accumulated_nudge[0]
            py += base_node.accumulated_nudge[1]
            pz += base_node.accumulated_nudge[2]

        ee_trans.transform.translation.x = float(px)
        ee_trans.transform.translation.y = float(py)
        ee_trans.transform.translation.z = float(pz)
        ee_trans.transform.rotation.w = float(qw)
        ee_trans.transform.rotation.x = float(qx)
        ee_trans.transform.rotation.y = float(qy)
        ee_trans.transform.rotation.z = float(qz)
        
        base_node.broadcaster.sendTransform(ee_trans)

    return next_deadline


class RobotBaseNode(Node):
    def __init__(self, env, num_envs):
        super().__init__('go2_driver_node')
        qos_profile = QoSProfile(depth=10)

        self.declare_parameter("lidar.include_extra_fields", True)
        self.declare_parameter("publish_map_odom", True)
        self.publish_map_odom = bool(self.get_parameter("publish_map_odom").value)

        self.env = env
        self.num_envs = num_envs
        self.qos_profile = qos_profile

        self._last_lin_vel_b = np.zeros((num_envs, 3), dtype=float)
        self._last_time_ns = None

        # --- MODIFIED: Arm Command Subscriber (Now expects Pose) ---
        self.arm_pose_target = None
        self.create_subscription(Pose, '/arm_commands', self._arm_cmd_cb, 10)

        # --- NEW: Return to Base Publisher ---
        self.return_pub = self.create_publisher(Empty, '/return_to_base', 10)

        self.camera_target_pose = None 
        self.create_subscription(Pose, '/camera_pose', self.camera_pose_cb, 10)
        self.cam_current_rot = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        # Video Recording State (Now using String IDs)
        self.is_recording = False
        self.video_rec_batch_id = "0"
        self.video_writer = None
        self.last_video_frame_time = 0.0
        # Updated topic name and message type to String
        self.create_subscription(String, '/record_video_str', self._record_video_req_cb, 10)

        self.photo_requested = False
        self.photo_batch_id = "0"
        # Updated topic name and message type to String
        self.create_subscription(String, '/photo_request_str', self._photo_req_cb, 10)

        import os
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.lidar_json_path = os.path.join(
            current_dir, "Isaac_sim", "Unitree", "Unitree_L1.json"
        )

        self.lidar_cfg = load_lidar_json(self.lidar_json_path)

        self.joint_pub = []
        self.go2_lidar_pub = []
        self.odom_pub = []
        self.imu_pub = []

        self.clock_pub = self.create_publisher(Clock, '/clock', qos_profile)

        # Odom Reset State
        self.odom_origins = {} 
        self._reset_odom_next_update = False

        # Accumulator for Map Nudging
        self.accumulated_nudge = np.array([0.0, 0.0, 0.0])

        for i in range(num_envs):
            self.joint_pub.append(self.create_publisher(JointState, '/joint_states', qos_profile))
            self.go2_lidar_pub.append(self.create_publisher(PointCloud2, '/glim_rosnode/points', qos_profile))
        if self.publish_map_odom:
            self._ensure_odom_publishers()
        self.broadcaster= TransformBroadcaster(self, qos=qos_profile)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        if self.publish_map_odom:
            self._publish_static_map_odom()
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _ensure_odom_publishers(self):
        while len(self.odom_pub) < self.num_envs:
            self.odom_pub.append(self.create_publisher(Odometry, 'odom', self.qos_profile))

    def _destroy_odom_publishers(self):
        for publisher in self.odom_pub:
            self.destroy_publisher(publisher)
        self.odom_pub.clear()

    def _reset_static_broadcaster(self):
        try:
            self.destroy_publisher(self.static_broadcaster.pub_tf)
        except Exception as exc:
            self.get_logger().warn(f"Failed to reset static TF publisher: {exc}")
        self.static_broadcaster = StaticTransformBroadcaster(self)

    def _on_set_parameters(self, parameters):
        for param in parameters:
            if param.name != "publish_map_odom":
                continue
            self.publish_map_odom = bool(param.value)
            if self.publish_map_odom:
                self._ensure_odom_publishers()
                self._publish_static_map_odom()
                self.get_logger().info("Enabled sim odom/map TF publishing")
            else:
                self._destroy_odom_publishers()
                self._reset_static_broadcaster()
                self.get_logger().info("Disabled sim odom/map TF publishing")
        return SetParametersResult(successful=True)
    
    def _arm_cmd_cb(self, msg):
        """Stores incoming ROS 2 Cartesian pose targets for the arm (relative to base)."""
        import numpy as np
        # Extract relative position (x, y, z)
        pos = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float32)
        # Extract relative orientation as quaternion (w, x, y, z)
        rot = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z], dtype=np.float32)
        
        self.arm_pose_target_rel = (pos, rot)
        
        # --- NEW: Print to console so we know the message arrived ---
        self.get_logger().info(f"Arm IK Target Received: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")

    # --- NEW: Helper method to publish the static transform ---
    def _publish_static_map_odom(self):
        if not self.publish_map_odom:
            return

        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = self.get_clock().now().to_msg()
        # Standard ROS REP-105: map is parent, odom is child
        tf_map_odom.header.frame_id = "odom"
        tf_map_odom.child_frame_id = "map"
        
        tf_map_odom.transform.translation.x = 0.0
        tf_map_odom.transform.translation.y = 0.0
        tf_map_odom.transform.translation.z = 0.0
        
        tf_map_odom.transform.rotation.x = 0.0
        tf_map_odom.transform.rotation.y = 0.0
        tf_map_odom.transform.rotation.z = 0.0
        tf_map_odom.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(tf_map_odom)
        self.get_logger().info("Published static transform: odom -> map")

    def _photo_req_cb(self, msg):
        # msg.data is now a string like "1" or "1.1"
        self.get_logger().info(f"Photo request received for folder: {msg.data}")
        self.photo_requested = True
        self.photo_batch_id = msg.data

    def _record_video_req_cb(self, msg):
        if not self.is_recording:
            # Start Recording
            self.is_recording = True
            self.video_rec_batch_id = msg.data
            self.video_writer = None  
            self.last_video_frame_time = 0.0
            self.get_logger().info(f"VIDEO START. Folder: {msg.data}")
        else:
            # Stop Recording
            self.is_recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            self.get_logger().info(f"VIDEO STOP. Finalized folder: {self.video_rec_batch_id}")

    def camera_pose_cb(self, msg):
        self.camera_target_pose = msg

    
    # --- NEW: Trigger function called by keyboard event ---
    def trigger_return_to_base(self):
        msg = Empty()
        self.return_pub.publish(msg)
        self.get_logger().info("Published request to /return_to_base")


    # --- NEW: Trigger function called by external script ---
    def trigger_odom_reset(self):
        """Signals that the next odometry update should re-zero the origin."""
        self.get_logger().info("ODOM RESET TRIGGERED")
        self._reset_odom_next_update = True

        if hasattr(self, 'accumulated_nudge'):
            self.accumulated_nudge[:] = 0.0

    # --- NEW: Nudge method called by sim ---
    def nudge_odom(self, dx, dy, dz):
        """Adds a positional offset to the odometry to match map movement."""
        # We add the delta to the accumulator. 
        # If the map moves +X, the robot appears to be at +X relative to the old map origin.
        self.accumulated_nudge += np.array([dx, dy, dz])
        self.get_logger().info(f"Odom Nudge: {self.accumulated_nudge}")

    def _publish_imus(self, time_msg):
        # ... (Unchanged) ...
        now_ns = int(time_msg.sec * 1_000_000_000 + time_msg.nanosec)
        robot = self.env.unwrapped.scene["robot"].data
        v_b = np.stack([robot.root_lin_vel_b[i, :].cpu().numpy() for i in range(self.num_envs)], axis=0)

        if self._last_time_ns is None:
            self._last_time_ns = now_ns
            self._last_lin_vel_b[:] = v_b
            return

        dt = (now_ns - self._last_time_ns) * 1e-9
        if dt <= 0.0:
            return 

        a_b = (v_b - self._last_lin_vel_b) / dt
        g_world = np.array([0.0, 0.0, -9.81])

        for i in range(self.num_envs):
            q = robot.root_state_w[i, 3:7]
            R_wb = Rotation.from_quat([q[1].item(), q[2].item(), q[3].item(), q[0].item()])
            R_bw = R_wb.inv() 

            g_body = R_bw.apply(g_world)    
            f_specific = a_b[i] - g_body      

            ang_vel_b = robot.root_ang_vel_b[i, :].cpu().numpy()
            self.publish_imu(q, f_specific, ang_vel_b, i, time_msg)

        self._last_time_ns = now_ns
        self._last_lin_vel_b[:] = v_b
        
    def publish_joints(self, joint_names_lst, joint_state_lst, robot_num, time_now):        # ... (Unchanged) ...
        joint_state = JointState()
        joint_state.header.stamp = time_now

        joint_state_names_formated = []
        for joint_name in joint_names_lst:
            joint_state_names_formated.append(joint_name)

        joint_state_formated = []
        for joint_state_val in joint_state_lst:
            joint_state_formated.append(joint_state_val.item())

        joint_state.name = joint_state_names_formated
        joint_state.position = joint_state_formated
        self.joint_pub[robot_num].publish(joint_state)

    def publish_odom(self, base_pos, base_rot, robot_num, time_now):
        # Base inputs are Tensor or Array from Isaac (World Frame)
        # base_pos: [x, y, z]
        # base_rot: [w, x, y, z] (Quat)

        # Convert to numpy for math
        current_pos = np.array([base_pos[0].item(), base_pos[1].item(), base_pos[2].item()])
        # Create Scipy rotation. Note: Scipy expects [x, y, z, w], Isaac/ROS often give w first or last.
        # This assumes base_rot is [w, x, y, z] based on context
        current_rot = Rotation.from_quat([base_rot[1].item(), base_rot[2].item(), base_rot[3].item(), base_rot[0].item()])

        # Convert robot base pose -> LiDAR/livox_frame pose.
        # PointCloud2 is published in frame_id="livox_frame", so odom should track this frame.
        lidar_pos = current_pos + current_rot.apply(LIVOX_OFFSET_BASE)
        lidar_rot = current_rot

        # --- Handle Odom Origin Logic ---
        # At startup, make odom coincident with the robot base frame.
        # On manual reset, re-zero odom to the current base frame again.
        if self._reset_odom_next_update or robot_num not in self.odom_origins:
            
            # Remove pitch/roll from the odom origin but KEEP the original spawn height.
            euler_zyx = current_rot.as_euler('zyx')
            flat_rot = Rotation.from_euler('zyx', [euler_zyx[0], 0.0, 0.0])

            self.odom_origins[robot_num] = {
                "pos": current_pos.copy(),
                "rot": flat_rot,
            }

            if robot_num == self.num_envs - 1:
                self._reset_odom_next_update = False

        # --- Calculate Relative Pose ---
        if robot_num in self.odom_origins:
            origin = self.odom_origins[robot_num]
            
            # 1. Get rotation difference: R_rel = R_origin_inv * R_current
            # R_rel is the robot's rotation relative to its starting orientation.
            R_rel = origin["rot"].inv() * lidar_rot
            
            # 2. Get translation difference in the Origin's frame
            # This preserves the lidar's true height above the robot base.
            P_diff = lidar_pos - origin["pos"]
            P_rel = origin["rot"].inv().apply(P_diff)
            
            # Update variables to be published
            final_pos = P_rel
            final_quat = R_rel.as_quat() # returns [x, y, z, w]
            
            # Map back to specific variables
            px, py, pz = final_pos[0], final_pos[1], final_pos[2]
            qx, qy, qz, qw = final_quat[0], final_quat[1], final_quat[2], final_quat[3]
        else:
            # Fallback: No reset occurred yet, publish absolute
            px, py, pz = current_pos[0], current_pos[1], current_pos[2]
            qw, qx, qy, qz = base_rot[0].item(), base_rot[1].item(), base_rot[2].item(), base_rot[3].item()

        # --- NEW: Apply Accumulated Nudge ---
        # We add the nudge to the final calculated position
        px += self.accumulated_nudge[0]
        py += self.accumulated_nudge[1]
        pz += self.accumulated_nudge[2]

        if not self.publish_map_odom:
            return

        # --- Publish (Updated with calculated values) ---
        odom_trans = TransformStamped()
        odom_trans.header.stamp = time_now
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "livox_frame"
        odom_trans.transform.translation.x = float(px)
        odom_trans.transform.translation.y = float(py)
        odom_trans.transform.translation.z = float(pz)
        odom_trans.transform.rotation.x = float(qx)
        odom_trans.transform.rotation.y = float(qy)
        odom_trans.transform.rotation.z = float(qz)
        odom_trans.transform.rotation.w = float(qw)
        self.broadcaster.sendTransform(odom_trans)

        odom_topic = Odometry()
        odom_topic.header.stamp = time_now
        odom_topic.header.frame_id = "odom"
        odom_topic.child_frame_id = "livox_frame"
        odom_topic.pose.pose.position.x = float(px)
        odom_topic.pose.pose.position.y = float(py)
        odom_topic.pose.pose.position.z = float(pz)
        odom_topic.pose.pose.orientation.x = float(qx)
        odom_topic.pose.pose.orientation.y = float(qy)
        odom_topic.pose.pose.orientation.z = float(qz)
        odom_topic.pose.pose.orientation.w = float(qw)
        self.odom_pub[robot_num].publish(odom_topic)

    def publish_imu(self, base_rot_wxyz, accel_specific_b, ang_vel_b, robot_num, time_now):
        if robot_num >= len(self.imu_pub):
            return

        # ... (Unchanged) ...
        try:
            q_w = float(getattr(base_rot_wxyz[0], "item", lambda: base_rot_wxyz[0])())
            q_x = float(getattr(base_rot_wxyz[1], "item", lambda: base_rot_wxyz[1])())
            q_y = float(getattr(base_rot_wxyz[2], "item", lambda: base_rot_wxyz[2])())
            q_z = float(getattr(base_rot_wxyz[3], "item", lambda: base_rot_wxyz[3])())
        except Exception:
            q_w, q_x, q_y, q_z = 1.0, 0.0, 0.0, 0.0 

        n = (q_w*q_w + q_x*q_x + q_y*q_y + q_z*q_z) ** 0.5
        if not np.isfinite(n) or n < 1e-9:
            q_w, q_x, q_y, q_z = 1.0, 0.0, 0.0, 0.0
        else:
            q_w, q_x, q_y, q_z = q_w/n, q_x/n, q_y/n, q_z/n

        msg = Imu()
        msg.header.stamp = time_now
        msg.header.frame_id = "livox_frame"

        msg.orientation.w = q_w
        msg.orientation.x = q_x
        msg.orientation.y = q_y
        msg.orientation.z = q_z

        msg.angular_velocity.x = float(ang_vel_b[0])
        msg.angular_velocity.y = float(ang_vel_b[1])
        msg.angular_velocity.z = float(ang_vel_b[2])

        msg.linear_acceleration.x = float(accel_specific_b[0])
        msg.linear_acceleration.y = float(accel_specific_b[1])
        msg.linear_acceleration.z = float(accel_specific_b[2])

        msg.orientation_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]
        msg.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        msg.linear_acceleration_covariance = [1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]

        self.imu_pub[robot_num].publish(msg)


def publish_lidar(self, points, robot_num, time_now, channels=None, rel_time=None, intensity_arr=None):
    # ... (Unchanged) ...
    include_extra = bool(self.get_parameter("lidar.include_extra_fields").get_parameter_value().bool_value)

    header = Header()
    header.stamp = time_now
    header.frame_id = "livox_frame"

    pts = np.asarray(points, dtype=np.float32)
    N = pts.shape[0]

    if DISABLE_RING_AND_TIME:
        intensity = np.asarray(intensity_arr, dtype=np.float32) if intensity_arr is not None else np.ones(N, dtype=np.float32)
        header = Header()
        header.stamp = time_now
        header.frame_id = "livox_frame"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_data = [
            (float(pts[i,0]), float(pts[i,1]), float(pts[i,2]), float(intensity[i]))
            for i in range(N)
        ]
        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
        self.go2_lidar_pub[robot_num].publish(cloud_msg)
        return  

    if channels is not None and len(channels) == N:
        rings = np.asarray(channels, dtype=np.uint16)
    else:
        rings = compute_ring_from_elevation(pts, self.lidar_cfg["elev_deg"])

    if rel_time is not None and len(rel_time) == N:
        t_rel = np.asarray(rel_time, dtype=np.float32)
    else:
        t_rel = compute_time_from_azimuth(
            pts,
            self.lidar_cfg["scan_period"],
            ring_idx=rings,
            ring_fine_offset_s=self.lidar_cfg["fire_time_s"]
        )

    if intensity_arr is not None and len(intensity_arr) == N:
        intens = np.asarray(intensity_arr, dtype=np.float32)
    else:
        intens = np.ones((N,), dtype=np.float32)

    if include_extra:
        fields = [
            PointField(name='x',          offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',          offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',          offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity',  offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',       offset=16, datatype=PointField.UINT16,  count=1),
            PointField(name='_pad',       offset=18, datatype=PointField.UINT16,  count=1),
            PointField(name='time',       offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='elevation',  offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring_time',  offset=28, datatype=PointField.FLOAT32, count=1),
        ]
        elev_deg = (
            self.lidar_cfg["elev_deg"][rings.astype(int)]
            if self.lidar_cfg["elev_deg"].size
            else np.degrees(np.arctan2(pts[:,2], np.sqrt(pts[:,0]**2 + pts[:,1]**2) + 1e-9)).astype(np.float32)
        )
        fire = self.lidar_cfg.get("fire_time_s", None)
        if fire is not None and np.size(fire) > 0:
            ring_time = (t_rel - fire[rings.astype(int)]) % self.lidar_cfg["scan_period"]
        else:
            ring_time = t_rel

        cloud_data = [
            (float(pts[i,0]), float(pts[i,1]), float(pts[i,2]),
             float(intens[i]), int(rings[i]), 0,
             float(t_rel[i]), float(elev_deg[i]), float(ring_time[i]))
            for i in range(N)
        ]
    else:
        fields = [
            PointField(name='x',          offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',          offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',          offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity',  offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',       offset=16, datatype=PointField.UINT16,  count=1),
            PointField(name='_pad',       offset=18, datatype=PointField.UINT16,  count=1),
            PointField(name='time',       offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_data = [
            (float(pts[i,0]), float(pts[i,1]), float(pts[i,2]),
             float(intens[i]), int(rings[i]), 0, float(t_rel[i]))
            for i in range(N)
        ]

    cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
    self.go2_lidar_pub[robot_num].publish(cloud_msg)
