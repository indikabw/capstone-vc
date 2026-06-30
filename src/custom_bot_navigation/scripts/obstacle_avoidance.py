#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd = Twist()
        self.start_time = self.get_clock().now()
        
        # Simple states: 0 = Move Forward, 1 = Turn Left, 2 = Turn Right
        self.state = 0
        self.state_start_time = self.get_clock().now()
        
        self.min_front_dist = 10.0
        self.min_left_dist = 10.0
        self.min_right_dist = 10.0

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        if num_ranges == 0:
            return
            
        # Extract segments
        front_ranges = ranges[-20:] + ranges[:20]
        left_ranges = ranges[45:135]
        right_ranges = ranges[-135:-45]
        
        self.min_front_dist = min([r for r in front_ranges if r > 0.1] + [10.0])
        self.min_left_dist = min([r for r in left_ranges if r > 0.1] + [10.0])
        self.min_right_dist = min([r for r in right_ranges if r > 0.1] + [10.0])

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        if elapsed > 120.0: # Stop after 2 minutes of sim time
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)
            self.get_logger().info('Navigation completed.')
            rclpy.shutdown()
            return

        state_elapsed = (now - self.state_start_time).nanoseconds / 1e9
        
        # State transitions
        if self.state == 0: # Move Forward
            if self.min_front_dist < 0.6:
                if self.min_left_dist > self.min_right_dist:
                    self.state = 1 # Turn Left
                else:
                    self.state = 2 # Turn Right
                self.state_start_time = now
        elif self.state == 1 or self.state == 2: # Turning
            # Keep turning until front is clear, plus a little extra time to face open space
            if self.min_front_dist > 1.2 and state_elapsed > 1.0:
                self.state = 0 # Move Forward
                self.state_start_time = now
                
        # State actions
        if self.state == 0:
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
        elif self.state == 1:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5
        elif self.state == 2:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -0.5
            
        self.publisher.publish(self.cmd)
        
        if int(elapsed * 10) % 20 == 0:
            self.get_logger().info(f'State: {self.state}, Distances: F:{self.min_front_dist:.2f} L:{self.min_left_dist:.2f} R:{self.min_right_dist:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
