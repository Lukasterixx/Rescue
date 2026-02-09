import os
import numpy as np
import math

import py_trees
from py_trees.common import Status

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid, MapMetaData


class FlattenPointCloudToMap2D(py_trees.behaviour.Behaviour):
    """
    Behaviour that listens to a PointCloud2 on /map, flattens them into
    a persistent 2D occupancy grid, and only updates a 20 m radius around
    the robot's current pose each publish cycle.
    It publishes the saved map immediately on startup (in setup) and
    uses transient-local QoS on /map2d so RViz can latch it.
    """

    def __init__(
        self,
        name: str,
        node: Node,
        resolution: float = 0.1,
        max_width_m: float = 200.0,
        max_height_m: float = 200.0,
        ground_height_threshold: float = 0.2,
        cloud_topic: str = '/map',
        pose_topic: str = '/current_pose',
        map2d_topic: str = '/map2d',
        queue_size: int = 10,
        publish_interval: float = 5.0,
        update_radius_m: float = 20.0,
    ):
        super().__init__(name)
        self.node = node

        # grid parameters
        self.resolution = resolution
        self.max_width_m = max_width_m
        self.max_height_m = max_height_m
        self.ground_height_threshold = ground_height_threshold
        self.update_radius_m = update_radius_m

        # compute fixed global grid size & origin (centered on world 0,0)
        self.global_width = int(self.max_width_m / self.resolution) + 1
        self.global_height = int(self.max_height_m / self.resolution) + 1
        half_w = (self.global_width * self.resolution) / 2.0
        half_h = (self.global_height * self.resolution) / 2.0
        self.origin_x = -half_w
        self.origin_y = -half_h

        # topics
        self.cloud_topic = cloud_topic
        self.pose_topic = pose_topic
        self.map2d_topic = map2d_topic
        self.queue_size = queue_size
        self.publish_interval = publish_interval

        # runtime state
        self.cloud_msg = None
        self.current_pose = None
        self.last_publish_time = None
        self.frame_id = None  # to use in header

        # storage path
        self.map_file = os.path.join(os.path.dirname(__file__), 'map2d.npy')

        # rclpy handles
        self.pose_sub = None
        self.cloud_sub = None
        self.publisher = None

    def setup(self, **kwargs) -> bool:
        # Default QoS for subscriptions (volatile) to match publishers on /map and /current_pose
        # and transient-local QoS for /map2d topic so late subscribers get the last map
        latch_qos = QoSProfile(depth=1)
        latch_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latch_qos.reliability = ReliabilityPolicy.RELIABLE

        # subscribe with default QoS (use simple queue_size) to avoid mismatch warnings
        self.pose_sub = self.node.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            self.queue_size
        )
        self.cloud_sub = self.node.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._cloud_callback,
            self.queue_size
        )
        # publisher uses latched-style QoS
        self.publisher = self.node.create_publisher(
            OccupancyGrid,
            self.map2d_topic,
            latch_qos
        )

        # load or init grid, then immediately publish
        self._load_grid()
        self._publish_full_grid()
        return True

    def initialise(self) -> None:
        # reset timers
        self.cloud_msg = None
        self.last_publish_time = None
        # grid already loaded and published in setup

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg.pose
        # capture the frame_id from pose messages
        self.frame_id = msg.header.frame_id

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self.cloud_msg = msg
        # if we don't yet have a frame_id, grab from cloud
        if not self.frame_id:
            self.frame_id = msg.header.frame_id

    def update(self) -> Status:
        if self.cloud_msg is None or self.current_pose is None:
            return Status.RUNNING

        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self.last_publish_time and (now - self.last_publish_time) < self.publish_interval:
            return Status.RUNNING

        # read all points once
        all_pts = list(pc2.read_points(
            self.cloud_msg,
            field_names=('x','y','z'),
            skip_nans=True
        ))
        cx, cy = self.current_pose.position.x, self.current_pose.position.y

        # compute dynamic threshold from average z in 1m radius
        local_z = [z for x, y, z in all_pts if (x-cx)**2 + (y-cy)**2 <= 1.0]
        avg_z = sum(local_z)/len(local_z) if local_z else 0.0
        thresh = self.ground_height_threshold + avg_z

        # filter points by thresh and radius
        r2 = self.update_radius_m ** 2
        filtered = [ (x,y) for x,y,z in all_pts
                     if z >= thresh and (x-cx)**2+(y-cy)**2 <= r2 ]
        if not filtered:
            self.last_publish_time = now
            return Status.RUNNING

        # clear the circular patch around robot in grid
        ci = int((cx-self.origin_x)/self.resolution)
        cj = int((cy-self.origin_y)/self.resolution)
        ri = int(self.update_radius_m/self.resolution)
        i_min, i_max = max(ci-ri,0), min(ci+ri,self.global_width-1)
        j_min, j_max = max(cj-ri,0), min(cj+ri,self.global_height-1)
        yy, xx = np.ogrid[j_min:j_max+1, i_min:i_max+1]
        wx = xx*self.resolution + self.origin_x + self.resolution/2
        wy = yy*self.resolution + self.origin_y + self.resolution/2
        mask = (wx-cx)**2 + (wy-cy)**2 <= r2
        self.grid[j_min:j_max+1, i_min:i_max+1][mask] = 0

        # mark occupied
        for x,y in filtered:
            ix = int((x-self.origin_x)/self.resolution)
            iy = int((y-self.origin_y)/self.resolution)
            if 0<=ix<self.global_width and 0<=iy<self.global_height:
                self.grid[iy, ix] = 100

        # publish and save
        self._publish_full_grid()
        np.save(self.map_file, self.grid)
        # also export PGM
        pgm_path = os.path.join(os.path.dirname(__file__), 'map2d.pgm')
        img = (self.grid.astype(np.float32)/100*255).astype(np.uint8)
        img = np.flipud(img)
        with open(pgm_path, 'wb') as f:
            f.write(f"P5\n{self.global_width} {self.global_height}\n255\n".encode())
            f.write(img.tobytes())

        self.last_publish_time = now
        return Status.RUNNING

    def _load_grid(self):
        if os.path.isfile(self.map_file):
            arr = np.load(self.map_file)
            if arr.shape == (self.global_height, self.global_width):
                self.grid = arr.astype(np.int8)
                return
        self.grid = np.zeros((self.global_height, self.global_width), dtype=np.int8)

    def _publish_full_grid(self):
        og = OccupancyGrid()
        og.header.stamp = self.node.get_clock().now().to_msg()
        og.header.frame_id = self.frame_id or 'map'
        info = MapMetaData()
        info.resolution = float(self.resolution)
        info.width = self.global_width
        info.height = self.global_height
        info.origin.position.x = float(self.origin_x)
        info.origin.position.y = float(self.origin_y)
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0
        og.info = info
        og.data = self.grid.flatten().tolist()
        self.publisher.publish(og)

    def terminate(self, new_status: Status) -> None:
        pass
