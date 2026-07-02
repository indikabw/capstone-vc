import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from custom_bot_interfaces.action import ReasoningTask
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import threading
import time
import math
import os
import sys
import json
import asyncio

try:
    from google.adk.agents import Agent
except ImportError:
    print("FATAL: google.adk.agents is not installed.")
    sys.exit(1)

if 'GEMINI_API_KEY' not in os.environ:
    print("FATAL: GEMINI_API_KEY environment variable is not set.")
    sys.exit(1)

class ReasoningNode(Node):
    def __init__(self):
        super().__init__('reasoning_node')
        self.get_logger().info('Initializing ADK 2.0 Reasoning Node...')
        
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
        
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        self.heartbeat_timer = self.create_timer(
            0.1,
            self.heartbeat_callback,
            callback_group=self.callback_group
        )
        
        # Load semantic map once
        self.sem_map = self.load_semantic_map()
        
        self.agent = Agent(
            name="robot_agent",
            model="gemini-2.5-flash",
            instruction="""
    You are an autonomous robot assistant controlling a TurtleBot4 (radius 0.35m) with an OpenManipulator-X arm.
    You must execute spatial reasoning to find objects, navigate to them, and manipulate them.
    
    CRITICAL INSTRUCTIONS:
    1. Use `list_objects_tool` to see all available objects in the environment. Look for keywords matching the user's request.
    2. If a user asks you to go to a general room (e.g., 'kitchen', 'bedroom') or the 'center' of a room, you must deduce the area by finding multiple objects that belong there (e.g., Refrigerator, Oven, KitchenTable for kitchen). Calculate the center of the bounding box of these objects to approximate the center of the room. Do this by finding the minimum and maximum X and Y coordinates among the objects, and calculating the midpoint of those bounds: ((min_x + max_x) / 2, (min_y + max_y) / 2).
    3. Use `get_object_details_tool` to find the exact (x, y, yaw) of target objects.
    4. Before navigating, you MUST pick an empty coordinate to stand in. Do NOT just blindly add an offset to a target object, or use the exact room center if it is occupied.
    5. Use `get_nearby_objects_tool(x, y, radius)` to verify if your proposed destination is empty. Ensure no objects are within a 0.5m radius. Pass exactly 0.5 as the radius. If there are objects, pick another coordinate by perturbing the point. If you cannot find a safe coordinate after 3 attempts, you must return a final textual response explaining why the navigation is impossible.
    6. Finally, use `navigate_and_face_tool` providing your safe (robot_x, robot_y) coordinate AND the target object's (face_x, face_y) coordinate (or the room center if looking at the center of the room). The system will automatically calculate the angle so you look at the target.
    7. Use `pick_tool(object_id)` to pick up an object. You must be navigated near it and facing it first.
    8. Use `place_tool(x, y, z)` to place an object at a target 3D coordinate.
    """,
            tools=[self.list_objects_tool, self.get_object_details_tool, self.get_nearby_objects_tool, self.navigate_and_face_tool, self.pick_tool, self.place_tool],
        )
        try:
            from google.adk.runners import InMemoryRunner
            self.runner = InMemoryRunner(agent=self.agent, app_name="custom_bot")
        except ImportError:
            self.runner = None
            self.get_logger().error("Could not import InMemoryRunner from google.adk.runners")

    def load_semantic_map(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('custom_bot_reasoning')
            map_path = os.path.join(pkg_share, 'resource', 'semantic_map.json')
        except Exception:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            map_path = os.path.join(base_dir, 'resource', 'semantic_map.json')
            
        if not os.path.exists(map_path):
            self.get_logger().error(f"semantic_map.json not found at {map_path}")
            return {}
            
        with open(map_path, 'r') as f:
            return json.load(f)

    def heartbeat_callback(self):
        pass
        
    def image_callback(self, msg):
        with self.image_lock:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Failed to convert image: {e}')

    def list_objects_tool(self) -> str:
        """Returns a list of all object IDs present in the environment."""
        self.get_logger().info('Tool called: list_objects_tool()')
        return json.dumps(list(self.sem_map.keys()))

    def get_object_details_tool(self, object_id: str) -> str:
        """Returns the specific position (x, y) and orientation (yaw) of the requested object ID."""
        self.get_logger().info(f'Tool called: get_object_details_tool({object_id})')
        if object_id in self.sem_map:
            return json.dumps(self.sem_map[object_id])
        return f'{{"error": "Object {object_id} not found."}}'

    def get_nearby_objects_tool(self, x: float, y: float, radius: float) -> str:
        """Returns a list of objects and their coordinates that are within 'radius' meters of the point (x, y). Use this to check for collisions."""
        self.get_logger().info(f'Tool called: get_nearby_objects_tool({x}, {y}, {radius})')
        nearby = {}
        for obj_id, details in self.sem_map.items():
            ox = details["position"]["x"]
            oy = details["position"]["y"]
            dist = math.sqrt((ox - x)**2 + (oy - y)**2)
            if dist <= radius:
                nearby[obj_id] = {"distance": dist, "position": details["position"]}
        
        if not nearby:
            return "No objects found within the radius. Coordinate is open."
        return json.dumps(nearby)

    def navigate_and_face_tool(self, robot_x: float, robot_y: float, face_x: float, face_y: float) -> str:
        """Move the robot to (robot_x, robot_y) and automatically turn to face (face_x, face_y).
        
        Args:
            robot_x: The X coordinate for the robot to stand at.
            robot_y: The Y coordinate for the robot to stand at.
            face_x: The X coordinate of the object the robot should look at.
            face_y: The Y coordinate of the object the robot should look at.
        """
        self.get_logger().info(f'Tool called: navigate_and_face_tool(robot: {robot_x},{robot_y}, face: {face_x},{face_y})')
        
        # Calculate yaw to face target
        theta = math.atan2(face_y - robot_y, face_x - robot_x)
        
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return "Failed to connect to Nav2 action server."
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(robot_x)
        goal_msg.pose.pose.position.y = float(robot_y)
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        future = self._nav_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            return "Nav2 goal was rejected. You may have chosen a coordinate inside an obstacle."
            
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
            
        from action_msgs.msg import GoalStatus
        res = result_future.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            return f"Navigation failed. The planner may have found the path blocked or the coordinate is inside an obstacle. Pick a DIFFERENT empty coordinate and try again."
            
        return "Navigation succeeded. Reached destination and facing target."

    def execute_trajectory(self, client, joint_names, positions, time_sec=2.0):
        if not client.wait_for_server(timeout_sec=2.0):
            return False, "Failed to connect to trajectory action server."

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
        goal_msg.trajectory.points.append(point)

        future = client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, "Trajectory goal rejected."

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        res = result_future.result()
        if res.result.error_code != 0:
            return False, f"Trajectory failed with error code: {res.result.error_code}"
            
        return True, "Success"

    def pick_tool(self, object_id: str) -> str:
        """Executes a predefined trajectory to pick up the specified object."""
        self.get_logger().info(f'Tool called: pick_tool({object_id})')
        
        arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
        gripper_joints = ['omx_gripper_left_joint']
        
        # 1. Open gripper
        self.execute_trajectory(self._gripper_client, gripper_joints, [0.010], 1.0)
        time.sleep(1.0)
        
        # 2. Reach forward
        ok, msg = self.execute_trajectory(self._arm_client, arm_joints, [0.0, 0.5, 0.5, -1.0], 2.0)
        if not ok: return msg
        time.sleep(2.0)
        
        # 3. Close gripper
        self.execute_trajectory(self._gripper_client, gripper_joints, [-0.010], 1.0)
        time.sleep(1.0)
        
        # 4. Retreat (Home)
        ok, msg = self.execute_trajectory(self._arm_client, arm_joints, [0.0, -1.0, 0.3, 0.7], 2.0)
        if not ok: return msg
        time.sleep(2.0)
            
        return f"Successfully picked up {object_id}."

    def place_tool(self, x: float, y: float, z: float) -> str:
        """Executes a predefined trajectory to place an object at (x, y, z)."""
        self.get_logger().info(f'Tool called: place_tool({x}, {y}, {z})')
        
        arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
        gripper_joints = ['omx_gripper_left_joint']
        
        # 1. Reach forward
        ok, msg = self.execute_trajectory(self._arm_client, arm_joints, [0.0, 0.5, 0.5, -1.0], 2.0)
        if not ok: return msg
        time.sleep(2.0)
        
        # 2. Open gripper
        self.execute_trajectory(self._gripper_client, gripper_joints, [0.010], 1.0)
        time.sleep(1.0)
        
        # 3. Retreat (Home)
        ok, msg = self.execute_trajectory(self._arm_client, arm_joints, [0.0, -1.0, 0.3, 0.7], 2.0)
        if not ok: return msg
        time.sleep(2.0)
            
        return f"Successfully placed object at ({x}, {y}, {z})."

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
        try:
            from google.genai import types
            async def run_adk():
                resp_text = ""
                session = await self.runner.session_service.create_session(
                    app_name="custom_bot", user_id="robot"
                )
                async for event in self.runner.run_async(
                    user_id="robot",
                    session_id=session.id,
                    new_message=types.Content(role="user", parts=[types.Part.from_text(text=goal_handle.request.command)])
                ):
                    try:
                        if event.content and event.content.parts:
                            for p in event.content.parts:
                                if p.text:
                                    resp_text += p.text
                    except AttributeError:
                        if hasattr(event, 'output') and event.output:
                            resp_text += str(event.output)
                return resp_text

            # Run the ADK asyncio loop safely in this worker thread
            response = asyncio.run(run_adk())
            summary = str(response)
        except Exception as e:
            self.get_logger().error(f"ADK Agent failed: {e}")
            summary = f"Reasoning failed: {e}"
        
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
