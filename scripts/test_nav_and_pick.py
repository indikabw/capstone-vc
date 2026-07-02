#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_bot_interfaces.action import ReasoningTask
import time

class NavPickTester(Node):
    def __init__(self):
        super().__init__('nav_pick_tester')
        self._action_client = ActionClient(self, ReasoningTask, '/reasoning_task')
        self.get_logger().info('Tester node initialized.')

    def send_goal(self, command):
        self.get_logger().info('Waiting for /reasoning_task action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server connected. Sending goal...')

        goal_msg = ReasoningTask.Goal()
        goal_msg.command = command

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback (Current Stage): {feedback.current_stage}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Final Status: {status}')
        self.get_logger().info(f'Success: {result.success}')
        self.get_logger().info(f'Summary: {result.summary}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavPickTester()
    action_client.send_goal("Navigate to the red_cube and pick it up.")
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
