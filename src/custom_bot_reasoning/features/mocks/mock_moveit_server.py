import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from moveit_msgs.action import MoveGroup

class MockMoveItServer(Node):
    def __init__(self):
        super().__init__('mock_moveit_server')
        self._action_server = ActionServer(
            self,
            MoveGroup,
            'move_action',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing mock MoveIt2 goal...')
        goal_handle.succeed()
        result = MoveGroup.Result()
        result.error_code.val = 1 # SUCCESS
        return result
