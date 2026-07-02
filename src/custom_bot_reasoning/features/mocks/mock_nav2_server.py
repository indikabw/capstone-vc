import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose

class MockNav2Server(Node):
    def __init__(self):
        super().__init__('mock_nav2_server')
        self.received_goals = []
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing mock Nav2 goal...')
        self.received_goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result
