#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from builtin_interfaces.msg import Duration


class TwistMux(Node):
    def __init__(self):
        super().__init__('twist_mux')

        self.declare_parameter('priorities', ['keyboard', 'joy', 'autonomous'])

        self.priorities = self.get_parameter('priorities').get_parameter_value().string_array_value

        self.key_sub = self.create_subscription(
            Twist,
            'keyboard_vel',
            self.key_callback,
            10
        )

        self.joy_sub = self.create_subscription(
            Twist,
            'joy_vel',
            self.joy_callback,
            10
        )

        self.auto_sub = self.create_subscription(
            Twist,
            'autonomous_vel',
            self.auto_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.cmd_type_pub = self.create_publisher(
            String,
            'cmd_type',
            10
        )

        self.last_msgs = {
            'keyboard': None,
            'joy': None,
            'autonomous': None,
        }

        self.create_timer(0.5, self.cmd_pub)  # 10 Hz


    def key_callback(self, msg):
        self.last_msgs['keyboard'] = (msg, self.get_clock().now())

    def joy_callback(self, msg):
        self.last_msgs['joy'] = (msg, self.get_clock().now())
    
    def auto_callback(self, msg):
        self.last_msgs['autonomous'] = (msg, self.get_clock().now())

    def cmd_pub(self):
        now = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=0.5)

        for key in self.priorities:
            data = self.last_msgs[key]

            if data is not None and (now - data[1] < timeout):
                self.cmd_vel_pub.publish(data[0])
                self.cmd_type_pub.publish(String(data=key))
                break

def main(args=None):
    rclpy.init(args=args)
    node = TwistMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

