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
    while navigator.get_clock().now().nanoseconds < 5e9:
        time.sleep(0.1)
        
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
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
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
