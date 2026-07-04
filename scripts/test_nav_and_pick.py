#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_bot_interfaces.action import ReasoningTask
import time
import subprocess
import re

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

    def verify_physics(self):
        try:
            result = subprocess.run(["gz", "model", "-m", "red_cylinder", "-p"], capture_output=True, text=True, check=True)
            output = result.stdout
            
            # Extract z position from output
            # Output format looks like:
            # pose {
            #   position {
            #     x: -1.87
            #     y: -2.0
            #     z: 0.15
            #   }
            z_match = re.search(r'position\s*\{\s*x:[^\n]+\n\s*y:[^\n]+\n\s*z:\s*([\d\.\-]+)', output)
            
            if z_match:
                z = float(z_match.group(1))
                self.get_logger().info(f'Physics check: red_cylinder z = {z:.4f}')
                if z > 0.18:
                    return True
                else:
                    self.get_logger().error('Physics check failed: Object was not lifted!')
                    return False
            else:
                self.get_logger().error('Could not parse z coordinate from gz model output')
                return False
        except Exception as e:
            self.get_logger().error(f'Failed to run gz model: {e}')
            return False

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Final Status: {status}')
        
        # Verify physical success even if node claims success
        physically_successful = False
        if result.success:
            self.get_logger().info('Node claims success. Verifying with physics engine...')
            physically_successful = self.verify_physics()
        else:
            self.get_logger().info('Node claims failure.')

        self.get_logger().info(f'Success: {physically_successful}')
        self.get_logger().info(f'Summary: {result.summary}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavPickTester()
    action_client.send_goal("Pick up the red_cylinder in front of you.")
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
