#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

class MoveItClient(Node):
    def __init__(self):
        super().__init__('test_moveit_arm', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
    def send_goal(self):
        self.get_logger().info('Waiting for MoveIt2 action server...')
        self._action_client.wait_for_server()
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        # Ensure 'arm' matches the group name in your SRDF
        req.group_name = 'arm'
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        
        c = Constraints()
        joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
        targets = [0.0, -1.0, 0.3, 0.7]
        
        for j, t in zip(joints, targets):
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = t
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        req.goal_constraints.append(c)
        goal_msg.request = req
        
        self.get_logger().info('Sending goal to MoveIt2...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    client = MoveItClient()
    future = client.send_goal()
    rclpy.spin_until_future_complete(client, future)
    
    goal_handle = future.result()
    if not goal_handle.accepted:
        client.get_logger().error('Goal rejected by MoveIt2!')
        return
        
    client.get_logger().info('Goal accepted, waiting for execution result...')
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client, result_future)
    
    result = result_future.result().result
    if result.error_code.val == 1:
        client.get_logger().info('SUCCESS! Arm moved using MoveIt2.')
    else:
        client.get_logger().error(f'Failed with MoveIt error code: {result.error_code.val}')
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
