"""What the sim says and hears on ROS 2 (Isaac's bundled Humble bridge, Fast DDS).

Published: `/clock` (sim time), `/joint_states`, `/odom` and the `odom -> livox_frame` TF (re-zeroed at startup and
on every level load), a static `odom -> map`, the RTX lidar on `/glim_rosnode/points` (through OmniGraph), its IMU on
`/livox/imu`, and the wrist RealSense on the realsense container's topics (`realsense.py`).
Subscribed: `robot0/cmd_vel`; `/photo_request_str` / `/record_video_str`, which save the wrist camera's colour
images under `photos/<id>/`; and `/sim/level` (std_msgs/String), the level window's buttons for scripts: a level's
key (gravel, krails, square, cup, ...), `reset`, `new_cup`, `lie` or `stand`.

Ground truth, for judging a behaviour and never for driving one (the robot has no such topic): `/sim/cup_pose`, the
cup's pose in the world frame, where its origin is on its axis at its base.

What is **not** here any more: `/arm_commands` and `/camera_pose`. The arm is the D1's own business now -- it is
driven over its DDS protocol by whatever holds it (the behaviour tree, through `maps/arm_bridge.py --sim`), exactly
as on the robot -- and the camera is fixed to the wrist, as the real one is. The TF for the wrist camera
(`arm_camera_link`) comes from VIP-Rescue's arm runtime, from the arm's own feedback, on the robot and here alike.

The lidar graph is Rescue's `ros2.py`, unchanged: an RTX lidar at LIVOX_OFFSET_BASE on the trunk with the
Livox-like blended vertical layout. The IMU is not the old native IMU graph: Isaac never created that sensor under
the welded robot (it logs "Could not create Imu sensor prim" and reports success anyway, in the old sim too), so
`/livox/imu` came up empty. It is computed here instead, at the policy rate, from the trunk's state at the lidar:
orientation, angular velocity, and the specific force (acceleration less gravity, the lidar point's velocity
differenced), all in livox_frame, which is the trunk's orientation.
"""
from __future__ import annotations

import datetime
import importlib
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

LIVOX_OFFSET_BASE = np.array([0.3, 0.0, 0.25], dtype=float)
IMU_TOPIC = "/livox/imu"
IMU_FRAME_ID = "livox_frame"
GRAVITY_W = np.array([0.0, 0.0, -9.81])
PHOTO_ROOT = Path(__file__).resolve().parents[1] / "photos"

_keep_alive = []
_lidar_render_products = []
_lidar_debug = {"enabled": False, "writer": None}


# ------------------------------------------------------------------------------------------ lidar and IMU
def _update_app_once():
    try:
        importlib.import_module("omni.kit.app").get_app().update()
    except Exception:
        pass


def _set_attr_if_present(prim, names, value):
    for name in names:
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            attr.Set(value)
            return True
    return False


def _lidar_graph(robot_num, lidar_sensor, topic_name="/glim_rosnode/points"):
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": f"/ROS_RTX_LIDAR_{robot_num}", "evaluator_name": "execution",
         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RtxLidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("RtxLidarHelper.inputs:enabled", True),
                ("RtxLidarHelper.inputs:renderProductPath", lidar_sensor.get_render_product_path()),
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


def add_lidar(num_envs: int = 1, debug: bool = False) -> None:
    import carb
    import omni.timeline
    import omni.usd
    from isaacsim.sensors.rtx import LidarRtx

    carb.settings.get_settings().set_bool("/app/sensors/nv/lidar/outputBufferOnGPU", True)
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        _update_app_once()
    for i in range(num_envs):
        parent = f"/World/envs/env_{i}/Robot/base"
        lidar = LidarRtx(prim_path=f"{parent}/lidar_sensor", translation=tuple(LIVOX_OFFSET_BASE),
                         orientation=(1.0, 0.0, 0.0, 0.0), config_file_name="Example_Rotary")
        lidar.initialize()
        _keep_alive.append(lidar)
        prim = omni.usd.get_context().get_stage().GetPrimAtPath(lidar.prim_path)
        if not prim or not prim.IsValid():
            continue
        for name, value in {
            "omni:sensor:Core:reportRateBaseHz": 4000,   # 5x, to keep the point density at the faster scan
            "omni:sensor:Core:scanRateBaseHz": 50,       # 50 Hz: no motion smear in fast turns
            "omni:sensor:Core:nearRangeM": 0.6,
            "omni:sensor:Core:farRangeM": 30.0,
            "omni:sensor:Core:numberOfEmitters": 128,
            "omni:sensor:Core:numberOfChannels": 128,
        }.items():
            _set_attr_if_present(prim, (name,), value)
        # Livox-ish blended vertical layout: four jittered stacks of 32 from -15 to +45 deg.
        elevation = prim.GetAttribute("omni:sensor:Core:emitterState:s001:elevationDeg")
        if elevation and elevation.IsValid():
            base = np.linspace(-15.0, 45.0, 32).astype(np.float32)
            rng = np.random.default_rng(seed=42)
            stacks = [np.sort(np.clip(base + rng.normal(0.0, 0.45, 32).astype(np.float32), -15.0, 45.0))
                      for _ in range(4)]
            elevation.Set(np.concatenate(stacks).tolist())
        _lidar_graph(i, lidar)
        _lidar_render_products.append(lidar.get_render_product_path())
    _lidar_debug["enabled"] = debug
    if debug:
        _toggle_debug_writer(True)


def _toggle_debug_writer(on: bool):
    import omni.replicator.core as rep

    if _lidar_debug["writer"] is None:
        _lidar_debug["writer"] = rep.writers.get("RtxLidarDebugDrawPointCloud")
    if on:
        _lidar_debug["writer"].attach(_lidar_render_products)
    else:
        _lidar_debug["writer"].detach()


def toggle_lidar_debug_draw():
    if not _lidar_render_products:
        return
    _lidar_debug["enabled"] = not _lidar_debug["enabled"]
    _toggle_debug_writer(_lidar_debug["enabled"])
    print(f"[lidar] debug drawing {'on' if _lidar_debug['enabled'] else 'off'}", flush=True)


# ------------------------------------------------------------------------------------------ the node
def sim_time_msg(seconds: float):
    from builtin_interfaces.msg import Time

    msg = Time()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - int(seconds)) * 1e9)
    return msg


class SimNode:
    """The sim's ROS node, `go2_driver_node` as before, so VIP-Rescue's launch files still find it."""

    def __init__(self, num_envs: int, on_cmd_vel, on_level=None):
        import rclpy
        from rcl_interfaces.msg import SetParametersResult
        from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        self._TransformStamped, self._Odometry, self._JointState, self._Imu = TransformStamped, Odometry, JointState, Imu
        self._SetParametersResult = SetParametersResult
        self.node = rclpy.create_node("go2_driver_node")
        node = self.node
        qos = QoSProfile(depth=10)
        self.num_envs = num_envs
        node.declare_parameter("publish_map_odom", True)
        self.publish_map_odom = bool(node.get_parameter("publish_map_odom").value)
        self.clock_pub = node.create_publisher(Clock, "/clock", qos)
        self._Clock = Clock
        self.joint_pub = node.create_publisher(JointState, "/joint_states", qos)
        self.imu_pub = node.create_publisher(Imu, IMU_TOPIC, qos)
        self.cup_pub = node.create_publisher(PoseStamped, "/sim/cup_pose", qos)
        self._PoseStamped = PoseStamped
        self._last_imu = None           # (sim time, lidar point velocity in the world)
        self.odom_pub = node.create_publisher(Odometry, "odom", qos) if self.publish_map_odom else None
        self.tf = TransformBroadcaster(node, qos=qos)
        self._static = StaticTransformBroadcaster(node)
        self._StaticTransformBroadcaster = StaticTransformBroadcaster
        self.odom_origins: dict[int, dict] = {}
        self._rezero = False
        for i in range(num_envs):
            node.create_subscription(Twist, f"robot{i}/cmd_vel",
                                     lambda msg, i=i: on_cmd_vel(i, msg.linear.x, msg.linear.y, msg.angular.z), 10)
        # Photos and video from the wrist camera, saved under photos/<id>/.
        self.photo_id = None
        self.video_id = None
        self._video = None
        node.create_subscription(String, "/photo_request_str", self._photo, 10)
        node.create_subscription(String, "/record_video_str", self._record, 10)
        if on_level is not None:
            node.create_subscription(String, "/sim/level", lambda msg: on_level(msg.data.strip()), 10)
        if self.publish_map_odom:
            self._publish_map_odom()
        node.add_on_set_parameters_callback(self._on_parameters)

    # ------------------------------------------------------------------ parameters (disable_sim_odom)
    def _on_parameters(self, parameters):
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile

        for param in parameters:
            if param.name != "publish_map_odom":
                continue
            self.publish_map_odom = bool(param.value)
            if self.publish_map_odom:
                if self.odom_pub is None:
                    self.odom_pub = self.node.create_publisher(Odometry, "odom", QoSProfile(depth=10))
                self._publish_map_odom()
            else:
                if self.odom_pub is not None:
                    self.node.destroy_publisher(self.odom_pub)
                    self.odom_pub = None
                try:
                    self.node.destroy_publisher(self._static.pub_tf)
                except Exception:
                    pass
                self._static = self._StaticTransformBroadcaster(self.node)
            self.node.get_logger().info(f"sim odom/map TF publishing {'on' if self.publish_map_odom else 'off'}")
        return self._SetParametersResult(successful=True)

    def _publish_map_odom(self):
        t = self._TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "odom", "map"
        t.transform.rotation.w = 1.0
        self._static.sendTransform(t)

    def rezero_odom(self) -> None:
        """The next odometry update becomes the new origin (startup, and every level load)."""
        self._rezero = True
        self.forget_motion()

    # ------------------------------------------------------------------ per step
    def publish(self, sim_time_s: float, robot, cup=None) -> object:
        stamp = sim_time_msg(sim_time_s)
        clock = self._Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)
        data = robot.data
        for i in range(self.num_envs):
            joints = self._JointState()
            joints.header.stamp = stamp
            joints.name = list(data.joint_names)
            joints.position = [float(v) for v in data.joint_pos[i].detach().cpu().tolist()]
            self.joint_pub.publish(joints)
            self._odom(i, data.root_state_w[i, :3].detach().cpu().numpy(),
                       data.root_state_w[i, 3:7].detach().cpu().numpy(), stamp)
        self._imu(sim_time_s, data, stamp)
        if cup is not None:
            pose = self._PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, "world"
            p = cup.data.root_pos_w[0].detach().cpu().tolist()
            q = cup.data.root_quat_w[0].detach().cpu().tolist()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
            pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = q
            self.cup_pub.publish(pose)
        return stamp

    def forget_motion(self) -> None:
        """A teleport is not an acceleration: the next IMU sample starts afresh."""
        self._last_imu = None

    def _imu(self, t: float, data, stamp) -> None:
        quat = data.root_quat_w[0].detach().cpu().numpy()                 # w, x, y, z
        rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
        omega_w = data.root_ang_vel_w[0].detach().cpu().numpy()
        # The lidar point's velocity: the trunk's, plus the turn about it.
        v_w = data.root_lin_vel_w[0].detach().cpu().numpy() + np.cross(omega_w, rot.apply(LIVOX_OFFSET_BASE))
        accel_w = np.zeros(3)
        if self._last_imu is not None and t > self._last_imu[0]:
            accel_w = (v_w - self._last_imu[1]) / (t - self._last_imu[0])
        self._last_imu = (t, v_w)
        msg = self._Imu()
        msg.header.stamp, msg.header.frame_id = stamp, IMU_FRAME_ID
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = (float(v) for v in quat)
        omega_b = rot.inv().apply(omega_w)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in omega_b)
        force_b = rot.inv().apply(accel_w - GRAVITY_W)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in force_b)
        msg.orientation_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]
        msg.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        msg.linear_acceleration_covariance = [1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]
        self.imu_pub.publish(msg)

    def _odom(self, i, pos, quat_wxyz, stamp):
        rot = Rotation.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        lidar_pos = pos + rot.apply(LIVOX_OFFSET_BASE)
        if self._rezero or i not in self.odom_origins:
            # Level the origin but keep the spawn height, so the lidar's height above the base is preserved.
            yaw = rot.as_euler("zyx")[0]
            self.odom_origins[i] = {"pos": pos.copy(), "rot": Rotation.from_euler("zyx", [yaw, 0.0, 0.0])}
            if i == self.num_envs - 1:
                self._rezero = False
        origin = self.odom_origins[i]
        p = origin["rot"].inv().apply(lidar_pos - origin["pos"])
        q = (origin["rot"].inv() * rot).as_quat()
        if not self.publish_map_odom:
            return
        t = self._TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = "odom", "livox_frame"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in p)
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
         t.transform.rotation.w) = (float(v) for v in q)
        self.tf.sendTransform(t)
        odom = self._Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = "odom", "livox_frame"
        pose = odom.pose.pose
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in p)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (float(v) for v in q)
        if self.odom_pub is not None:
            self.odom_pub.publish(odom)

    # ------------------------------------------------------------------ photos and video
    def _photo(self, msg):
        self.photo_id = msg.data
        self.node.get_logger().info(f"photo requested into photos/{msg.data}")

    def _record(self, msg):
        if self._video is None and self.video_id is None:
            self.video_id = msg.data
            self.node.get_logger().info(f"video recording into photos/{msg.data}")
        else:
            if self._video is not None:
                self._video.release()
            self._video, self.video_id = None, None
            self.node.get_logger().info("video stopped")

    def save_images(self, rgb: np.ndarray, fps: float) -> None:
        """Photo and video requests, fed the wrist camera's colour frames as they are published."""
        import cv2

        if self.photo_id is not None:
            folder = PHOTO_ROOT / str(self.photo_id)
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = folder / f"photo_{self.photo_id}_{stamp}.png"
            cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            self.node.get_logger().info(f"saved {path}")
            self.photo_id = None
        if self.video_id is not None:
            if self._video is None:
                folder = PHOTO_ROOT / str(self.video_id)
                folder.mkdir(parents=True, exist_ok=True)
                height, width = rgb.shape[:2]
                self._video = cv2.VideoWriter(str(folder / f"video_{self.video_id}.mp4"),
                                              cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
            self._video.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    def close(self):
        if self._video is not None:
            self._video.release()
