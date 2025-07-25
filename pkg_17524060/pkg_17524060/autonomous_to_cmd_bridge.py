#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AutonomousToCmdBridge(Node):
    def __init__(self):
        super().__init__('autonomous_to_cmd_bridge')
        self.subscription = self.create_subscription(
            Twist,
            'autonomous_vel',
            self.listener_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_type_pub = self.create_publisher(String, 'cmd_type', 10)
        self.get_logger().info('Node started: relaying autonomous_vel -> cmd_vel and publishing "autonomous" to cmd_type')

    def listener_callback(self, msg):
        # Relay Twist message
        self.cmd_vel_pub.publish(msg)
        # Publish 'autonomous' to cmd_type
        cmd_type_msg = String()
        cmd_type_msg.data = 'autonomous'
        self.cmd_type_pub.publish(cmd_type_msg)
        self.get_logger().info('Relayed Twist to cmd_vel and published "autonomous" to cmd_type')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousToCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
