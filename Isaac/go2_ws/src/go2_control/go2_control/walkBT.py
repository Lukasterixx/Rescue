# go2_control/go2_control/walk_node.py

# ros2 run go2_control walk_forward_bt_parallel

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import py_trees
from py_trees.common import ParallelPolicy
from py_trees_ros.trees import BehaviourTree
import geometry_msgs.msg

# Import your two leaf behaviours (each defined in its own file)
from go2_control.permission_behavior import PermissionBehavior
from go2_control.walk_forward_behavior import WalkForwardBehavior
from go2_control.slam_launch_behavior import SLAMLaunchBehavior
from go2_control.flatten2d import FlattenPointCloudToMap2D

from go2_control.nav2 import (
    Nav2BringupBehavior,
    Nav2GoalBehavior
)

class WalkNode(Node):
    def __init__(self):
        super().__init__('walk_node')

        # --- 1) Create your publisher & Twist message:
        self.cmd_pub = self.create_publisher(Twist, 'robot0/cmd_vel', 10)
        twist_msg = Twist()
        twist_msg.linear.x = 1.0

        # --- 2) Instantiate your three leaf behaviours:
        permission   = PermissionBehavior(name="Permission")
        slam_launcher = SLAMLaunchBehavior(name="SLAMLauncher")
        flattener = FlattenPointCloudToMap2D(
            name="Flatten2DMap",
            node=self,
            resolution=0.25,
            ground_height_threshold=0.7,
            cloud_topic='/map',
            map2d_topic='/map2d',
            publish_interval=5.0
        )
        walk_forward = WalkForwardBehavior(
            name="WalkForward",
            publisher=self.cmd_pub,
            twist_msg=twist_msg
        )
        nav2_bringup  = Nav2BringupBehavior(name="Nav2Bringup")

        # Define a fixed or dynamic goal:
        goal = geometry_msgs.msg.PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 2.0
        goal.pose.position.y = 3.0
        goal.pose.orientation.w = 1.0

        nav2_goal     = Nav2GoalBehavior(
                            name="Nav2Goal",
                            goal_pose=goal
                        )

        # --- 3) Build the Parallel root with SuccessOnAll (no synchronisation) ---
        root = py_trees.composites.Parallel(
            name="MainParallel",
            policy=ParallelPolicy.SuccessOnAll(synchronise=False)
        )
        root.add_children([
            permission,
            slam_launcher,
            flattener,
            #nav2_bringup,
            #nav2_goal
        ])
        

        # --- 4) Wrap in a py_trees_ros BehaviourTree and start it ticking at 10 Hz ---
        self.tree = BehaviourTree(root=root)
        self.tree.setup(node=self, node_name='walk_node_tree')
        self.tree.tick_tock(period_ms=100)

        self.get_logger().info("WalkNode: Parallel tree started @ 10 Hz")


def main(args=None):
    rclpy.init(args=args)
    node = WalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # always clean up the node
        try:
            node.destroy_node()
        except Exception:
            pass
        # guard against double‐shutdown
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
