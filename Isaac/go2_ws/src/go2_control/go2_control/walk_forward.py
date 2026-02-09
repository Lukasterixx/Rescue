# walk_forward_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class WalkForwardNode(Node):
    def __init__(self):
        super().__init__('walk_forward_node')
        # Publisher on "unitree_go2/cmd_vel" (queue size 10)
        self.cmd_pub = self.create_publisher(Twist, 'robot0/cmd_vel', 10)

        # Prepare a constant Twist: move forward at 0.5 m/s
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.5  # forward speed (m/s)
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0  # no rotation

        # Publish at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('WalkForwardNode started, publishing to "robot0/cmd_vel"')

    def timer_callback(self):
        # Simply keep publishing the same forward‐walk command
        self.cmd_pub.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WalkForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
