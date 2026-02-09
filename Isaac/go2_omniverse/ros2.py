# Copyright (c) 2024, RoboVerse community
# ... (Header unchanged) ...

import asyncio
import time
import numpy as np
import cv2                 # OpenCV for video recording
import os, json, math

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

# --- Message Imports ---
from sensor_msgs.msg import JointState, PointCloud2, PointField, Imu
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import Header, String, Empty
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from go2_interfaces.msg import Go2State  # Ensure you have this custom interface package
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs_py import point_cloud2

# --- Omniverse / Pixar Imports ---
from pxr import Gf, UsdGeom
import omni.usd
import omni.replicator.core as rep
from omni.isaac.orbit.sensors import CameraCfg, Camera
from omni.isaac.sensor import LidarRtx
import omni.isaac.orbit.sim as sim_utils
from scipy.spatial.transform import Rotation
from PIL import Image
import datetime
from ament_index_python.packages import get_package_share_directory

DISABLE_RING_AND_TIME = True    # fast mode

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
    pts[:, 2] += 0.4
    return pts

def add_rtx_lidar(num_envs, robot_type, debug=False):
    annotator_lst = []
    for i in range(num_envs):
        if robot_type == "g1":
            lidar_sensor = LidarRtx(f'/World/envs/env_{i}/Robot/head_link/lidar_sensor',
                                    rotation_frequency = 200,
                                    pulse_time=1, 
                                    translation=(0.0, 0.0, 0.0),
                                    orientation=(1.0, 0.0, 0.0, 0.0),
                                    config_file_name= "Unitree_L1",
                                )
        else:
            lidar_sensor = LidarRtx(f'/World/envs/env_{i}/Robot/base/lidar_sensor',
                                    rotation_frequency = 10,
                                    pulse_time=1, 
                                    translation=(0.0, 0, 0.4),
                                    orientation=(1.0, 0.0, 0.0, 0.0),
                                    config_file_name= "Unitree_L1",
                                    )

        if debug:
            writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloudBuffer")
            writer.attach([lidar_sensor.get_render_product_path()])

        annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
        annotator.attach(lidar_sensor.get_render_product_path())
        annotator_lst.append(annotator)
    return annotator_lst

def add_camera(num_envs, robot_type):
    annotators = []
    _cameras_keep_alive = [] 

    for i in range(num_envs):
        cameraCfg = CameraCfg(
            prim_path=f"/World/envs/env_{i}/Robot/base/front_cam",
            update_period=0.1,
            height=512,
            width=640,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=13.0, 
                focus_distance=400.0, 
                horizontal_aperture=7.68,
                clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.42), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
        )

        if robot_type == "g1":
            cameraCfg.prim_path = f"/World/envs/env_{i}/Robot/head_link/front_cam"
            cameraCfg.offset = CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.5, -0.5, 0.5, -0.5), convention="ros")

        cam = Camera(cameraCfg)
        _cameras_keep_alive.append(cam)

        render_prod = rep.create.render_product(cameraCfg.prim_path, (640, 512))
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


def pub_robo_data_ros2(robot_type, num_envs, base_node, env, annotator_lst, next_deadline, camera_lst=None):
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
            cam_prim_path = f"/World/envs/env_{i}/Robot/base/front_cam"
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
    time_now_for_clock = base_node.get_clock().now().to_msg()
    clock_msg = Clock()
    clock_msg.clock = time_now_for_clock
    base_node.clock_pub.publish(clock_msg)

    for i in range(num_envs):
        base_node.publish_joints(
            env.env.scene["robot"].data.joint_names,
            env.env.scene["robot"].data.joint_pos[i],
            i
        )
        base_node.publish_odom(
            env.env.scene["robot"].data.root_state_w[i, :3],
            env.env.scene["robot"].data.root_state_w[i, 3:7],
            i,
            time_now_for_clock
        )

    # ---------- 10 Hz LiDAR gate ----------
    period = 0.1  
    now = time.monotonic()
    if now >= next_deadline:
        burst_stamp = time_now_for_clock
        try:
            for j in range(num_envs):
                data = annotator_lst[j].get_data()

                if j == 0 and not hasattr(base_node, "_printed_lidar_schema"):
                    base_node._printed_lidar_schema = True
                    base_node.get_logger().info(
                        f"LiDAR bundle keys: {list(data.keys())}, "
                        f"data.shape: {np.asarray(data.get('data')).shape if 'data' in data else 'n/a'}"
                    )

                pts = _extract_points(data)
                channels  = _extract_optional(data, "channel", "ring", "channels", "rings")
                rel_time  = _extract_optional(data, "relative_time", "timestamps", "time")
                intensity = _extract_optional(data, "intensity", "intensities", "reflectance")

                point_cloud = update_meshes_for_cloud2(
                    pts,
                    env.env.scene["robot"].data.root_state_w[j, :3],
                    env.env.scene["robot"].data.root_state_w[j, 3:7],
                )

                base_node._publish_imus()
                publish_lidar(
                    base_node, point_cloud, j, burst_stamp,
                    channels=channels, rel_time=rel_time, intensity_arr=intensity
                )

        except Exception as e:
            base_node.get_logger().warn(f"LiDAR burst skipped: {e}")

        next_deadline += period
        if now - next_deadline > period:
            next_deadline = now + period

    return next_deadline


class RobotBaseNode(Node):
    def __init__(self, env, num_envs):
        super().__init__('go2_driver_node')
        qos_profile = QoSProfile(depth=10)

        self.declare_parameter("lidar.include_extra_fields", True)

        self.env = env
        self.num_envs = num_envs

        self._last_lin_vel_b = np.zeros((num_envs, 3), dtype=float)
        self._last_time_ns = None

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

        self.lidar_json_path = os.path.expanduser(
            "~/P2Dingo/Isaac/go2_omniverse/Isaac_sim/Unitree/Unitree_L1.json"
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
            self.odom_pub.append(self.create_publisher(Odometry, 'odom', qos_profile))
            self.imu_pub.append(self.create_publisher(Imu, '/livox/imu', qos_profile))
        self.broadcaster= TransformBroadcaster(self, qos=qos_profile)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        self._publish_static_map_odom()

    # --- NEW: Helper method to publish the static transform ---
    def _publish_static_map_odom(self):
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

    def _publish_imus(self):
        # ... (Unchanged) ...
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        robot = self.env.env.scene["robot"].data
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

        time_msg = now.to_msg()
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
        
    def publish_joints(self, joint_names_lst, joint_state_lst, robot_num):
        # ... (Unchanged) ...
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        joint_state_names_formated = []
        for joint_name in joint_names_lst:
            joint_state_names_formated.append(f"robot{robot_num}/"+joint_name)

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

        # --- Handle Reset Logic ---
        if self._reset_odom_next_update:
            # [MODIFIED] Force the new origin to be on the ground (Z=0) and flat (Identity Rotation).
            # This prevents the odom frame from 'teleporting' up to the robot's height 
            # or adopting the robot's pitch/roll.
            self.odom_origins[robot_num] = {
                'pos': np.array([current_pos[0], current_pos[1], 0.0]), # Only capture X, Y. Z is fixed to 0.
                'rot': Rotation.identity()  # Enforce world-aligned (flat) frame, ignoring robot tilt.
            }
            
            # Only clear the flag after the last robot is processed
            if robot_num == self.num_envs - 1:
                self._reset_odom_next_update = False

        # --- Calculate Relative Pose ---
        if robot_num in self.odom_origins:
            origin = self.odom_origins[robot_num]
            
            # 1. Get rotation difference: R_rel = R_origin_inv * R_current
            # Since origin is now Identity, R_rel is just the robot's true world rotation (preserving Pitch/Roll).
            R_rel = origin['rot'].inv() * current_rot
            
            # 2. Get translation difference in the Origin's frame
            # Since origin Z is 0, this preserves the robot's true height above ground.
            P_diff = current_pos - origin['pos']
            P_rel = origin['rot'].inv().apply(P_diff)
            
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