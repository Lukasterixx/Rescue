#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry

import tf2_ros
from geometry_msgs.msg import TransformStamped


class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')

        # --- parameters ---
        self.declare_parameter('input_pointcloud', '/robot0/point_cloud2')
        self.declare_parameter('output_scan',     '/scan')
        self.declare_parameter('angle_min',       -math.pi)
        self.declare_parameter('angle_max',        math.pi)
        self.declare_parameter('angle_increment',  math.radians(1.0))
        self.declare_parameter('range_min',        0.0)
        self.declare_parameter('range_max',       30.0)
        self.declare_parameter('height_max', 0.60)  # meters below ground to skip
        self.declare_parameter('height_min', 0.30)  # meters above ground to skip

        
        self.height_max = self.get_parameter('height_max').value
        self.height_min = self.get_parameter('height_min').value

        pc_in    = self.get_parameter('input_pointcloud').value
        scan_out = self.get_parameter('output_scan').value
        self.angle_min       = self.get_parameter('angle_min').value
        self.angle_max       = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min       = self.get_parameter('range_min').value
        self.range_max       = self.get_parameter('range_max').value

        self.num_beams = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1


        # PointCloud → LaserScan
        self.scan_pub = self.create_publisher(LaserScan, scan_out, 10)
        self.pc_sub   = self.create_subscription(
            PointCloud2, pc_in, self.pc_callback, 10)

        self.last_msg_time = self.get_clock().now()


    def pc_callback(self, msg: PointCloud2):
        now = self.get_clock().now()
        dt  = (now - self.last_msg_time).nanoseconds * 1e-9
        self.last_msg_time = now

        # bin points into a 2D scan
        ranges = [float('inf')] * self.num_beams
        for x, y, z in point_cloud2.read_points(
                msg, field_names=('x','y','z'), skip_nans=True):
            
            # —— new: skip ground returns ——
            if not (self.height_min < abs(z) < self.height_max):
                continue
            r = math.hypot(x, y)
            if not (self.range_min < r < self.range_max):
                continue
            ang = math.atan2(y, x)
            idx = int((ang - self.angle_min) / self.angle_increment)
            if 0 <= idx < self.num_beams and r < ranges[idx]:
                ranges[idx] = r

        for i in range(self.num_beams):
            if ranges[i] == float('inf'):
                ranges[i] = self.range_max

        scan = LaserScan()
        scan.header.stamp    = now.to_msg()
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min       = self.angle_min
        scan.angle_max       = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment  = dt / self.num_beams
        scan.scan_time       = dt
        scan.range_min       = self.range_min
        scan.range_max       = self.range_max
        scan.ranges          = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
