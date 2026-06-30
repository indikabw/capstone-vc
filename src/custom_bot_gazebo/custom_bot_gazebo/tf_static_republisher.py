#!/usr/bin/env python3
"""
TF Static Republisher — Disabled.

Previously this node re-published /tf_static transforms onto /tf at 10Hz,
which caused constant "Detected jump back in time" warnings that flushed the
TF buffer and broke AMCL localisation.

robot_state_publisher already publishes all static transforms on /tf_static
with TRANSIENT_LOCAL QoS (latched), so any new subscriber immediately
receives the full transform tree. Republishing them on /tf as dynamic
transforms is both incorrect and harmful.

This node is now a no-op placeholder retained only for backward compatibility
with the launch file reference. It can be safely removed from sim.launch.py
if desired.
"""
import rclpy
from rclpy.node import Node


class TfStaticRepublisher(Node):
    def __init__(self):
        super().__init__('tf_static_republisher')
        self.get_logger().info(
            "TF Static Republisher: disabled. "
            "Static TFs are served correctly by robot_state_publisher on /tf_static."
        )


def main(args=None):
    rclpy.init(args=args)
    node = TfStaticRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
