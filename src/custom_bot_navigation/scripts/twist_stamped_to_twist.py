#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStampedToTwist(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')
        self.subscription = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_unstamped', 10)

    def listener_callback(self, msg):
        twist = Twist()
        twist.linear = msg.twist.linear
        twist.angular = msg.twist.angular
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
