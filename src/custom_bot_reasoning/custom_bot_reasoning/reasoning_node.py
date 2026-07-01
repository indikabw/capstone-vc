import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from custom_bot_interfaces.action import ReasoningTask
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import threading
import time
import math
import os
import json

try:
    from google.adk.agents import Agent
except ImportError:
    Agent = None

if 'GEMINI_API_KEY' not in os.environ:
    Agent = None

class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')
        self.get_logger().info('Initializing Reasoning Node...')
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        self._action_server = ActionServer(
            self,
            ReasoningTask,
            'reasoning_task',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )
        
        self._nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose', 
            callback_group=self.callback_group
        )
        
        self.heartbeat_timer = self.create_timer(
            0.1,
            self.heartbeat_callback,
            callback_group=self.callback_group
        )
        
        if Agent is not None:
            self.agent = Agent(
                name="robot_agent",
                model="gemini-1.5-flash",
                instruction="""
        You are an autonomous robot assistant. You can see the environment and move to specific coordinates.
        Use the get_semantic_map tool to find the layout of the environment and the objects in it. You can reason about which room an object is in based on the layout of other objects.
        
        When determining a coordinate to move to, make sure the destination is open enough. The TurtleBot4 has a radius of approximately 0.35m.
        Use the target object's position and orientation to calculate a safe offset (e.g., 0.5m to 1.0m away) so the robot does not collide with the object.
        
        Analyze the user command and use the navigate_to_pose tool to move to the appropriate location.
        """,
                tools=[self.navigate_to_pose_tool, self.get_semantic_map_tool],
            )
        else:
            self.agent = None
            self.get_logger().warning('google.adk.agents not available, running in mock mode.')
        
    def heartbeat_callback(self):
        pass
        
    def image_callback(self, msg):
        with self.image_lock:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Failed to convert image: {e}')

    def get_semantic_map_tool(self) -> str:
        """Returns a JSON string containing the semantic map of the environment, including object positions and orientations."""
        self.get_logger().info('Tool called: get_semantic_map()')
        
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('custom_bot_reasoning')
            map_path = os.path.join(pkg_share, 'resource', 'semantic_map.json')
        except Exception:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            map_path = os.path.join(base_dir, 'resource', 'semantic_map.json')
            
        if not os.path.exists(map_path):
            return '{"error": "semantic_map.json not found"}'
            
        with open(map_path, 'r') as f:
            return f.read()

    def navigate_to_pose_tool(self, x: float, y: float, theta: float) -> str:
        """Move the robot to the specified 2D coordinate on the map.
        
        Args:
            x: X coordinate in meters.
            y: Y coordinate in meters.
            theta: Yaw angle in radians.
        """
        self.get_logger().info(f'Tool called: navigate_to_pose(x={x}, y={y}, theta={theta})')
        
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return "Failed to connect to Nav2 action server."
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        future = self._nav_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            return "Nav2 goal was rejected."
            
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
            
        return "Navigation succeeded."

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received reasoning goal: "{goal_handle.request.command}"')
        
        feedback_msg = ReasoningTask.Feedback()
        feedback_msg.current_stage = "sampling image"
        goal_handle.publish_feedback(feedback_msg)
        
        current_img = None
        with self.image_lock:
            if self.latest_image is not None:
                current_img = self.latest_image.copy()
                
        if current_img is None:
            self.get_logger().warning('No image received yet on /camera/image_raw')
            
        feedback_msg.current_stage = "reasoning"
        goal_handle.publish_feedback(feedback_msg)
        
        summary = ""
        if self.agent is not None:
            try:
                response = self.agent(goal_handle.request.command)
                summary = str(response)
            except Exception as e:
                self.get_logger().error(f"ADK Agent failed: {e}")
                summary = f"Reasoning failed: {e}"
        else:
            self.get_logger().info('Mocking ADK reasoning loop.')
            cmd = goal_handle.request.command.lower()
            if "coffee_table" in cmd or "living_room" in cmd:
                self.navigate_to_pose_tool(1.5, -0.5, -1.57)
            elif "kitchen" in cmd:
                self.navigate_to_pose_tool(5.5, 1.0, 0.0)
            elif "bedroom" in cmd:
                self.navigate_to_pose_tool(-5.0, 2.0, 3.14)
            summary = "Mock reasoning complete."
        
        self.get_logger().info('Reasoning complete.')
        goal_handle.succeed()
        
        result = ReasoningTask.Result()
        result.success = True
        result.summary = summary
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ReasoningNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
