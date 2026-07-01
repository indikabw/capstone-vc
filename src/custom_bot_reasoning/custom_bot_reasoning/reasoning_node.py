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
                instruction=(
                    "You are an autonomous robot assistant. You can see the environment "
                    "and move to specific coordinates. "
                    "Semantic Map: \n"
                    "- kitchen: x=1.5, y=0.5, theta=0.0\n"
                    "- living_room: x=-1.0, y=1.0, theta=1.57\n"
                    "- bedroom: x=0.0, y=-2.0, theta=-1.57\n"
                    "Analyze the user command and use the navigate_to_pose tool to move to the appropriate location."
                ),
                tools=[self.navigate_to_pose_tool],
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
            if "coffee_table" in cmd:
                self.navigate_to_pose_tool(2.0, 2.0, 0.0)
            elif "kitchen" in cmd:
                self.navigate_to_pose_tool(1.5, 0.5, 0.0)
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
