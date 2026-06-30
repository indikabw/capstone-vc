#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import sys
import threading

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    
    navigator = BasicNavigator()

    print("Waiting for simulation time to advance past 5 seconds (TF buffer fill)...")
    temp_node = Node('clock_waiter', parameter_overrides=[rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    while temp_node.get_clock().now().nanoseconds < 5e9:
        rclpy.spin_once(temp_node, timeout_sec=0.1)
    temp_node.destroy_node()
        
    print("Setting initial pose...")
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp.sec = 0
    initial_pose.header.stamp.nanosec = 0
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    print("Sending goal to x=4.0, y=4.0 (should require navigating around obstacles)...")
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    # Use zero timestamp so TF lookup uses latest available transform,
    # avoiding extrapolation errors under the slow llvmpipe simulation clock.
    goal_pose.header.stamp.sec = 0
    goal_pose.header.stamp.nanosec = 0
    goal_pose.pose.position.x = 4.0
    goal_pose.pose.position.y = 4.0
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
        time.sleep(1.0)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("SUCCESS! Robot reached the destination.")
        sys.exit(0)
    elif result == TaskResult.CANCELED:
        print("FAILED! Navigation was canceled.")
        sys.exit(1)
    elif result == TaskResult.FAILED:
        print("FAILED! Navigation failed to find path or execute.")
        sys.exit(1)
    else:
        print("FAILED! Unknown result.")
        sys.exit(1)

if __name__ == '__main__':
    main()
