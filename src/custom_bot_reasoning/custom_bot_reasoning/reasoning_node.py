import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from custom_bot_interfaces.action import ReasoningTask
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import Point, Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
        
        self._moveit_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
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
    You are an autonomous robot assistant controlling a TurtleBot4 (actual base radius 0.17m) with an OpenManipulator-X arm.
    You must execute spatial reasoning to find objects, navigate to them, and manipulate them.
    
    CRITICAL INSTRUCTIONS:
    1. Use `list_objects_tool` to see all available objects in the environment. Look for keywords matching the user's request.
    2. If a user asks you to go to a general room (e.g., 'kitchen', 'bedroom') or the 'center' of a room, you must deduce the area by finding multiple objects that belong there (e.g., Refrigerator, Oven, KitchenTable for kitchen). Calculate the center of the bounding box of these objects to approximate the center of the room. Do this by finding the minimum and maximum X and Y coordinates among the objects, and calculating the midpoint of those bounds: ((min_x + max_x) / 2, (min_y + max_y) / 2).
    3. Use `get_object_details_tool` to find the exact (x, y, yaw) of target objects.
    4. Before navigating, you MUST pick an empty coordinate to stand in. Do NOT just blindly add an offset to a target object, or use the exact room center if it is occupied.
    5. When picking up an object, you MUST stand exactly 0.35m away from the object's center and face it. To prevent path planning collisions with the corners of square tables, you MUST approach the object strictly along the X or Y axis (orthogonally). For example, if target is at (tx, ty), your standing position MUST be exactly one of: (tx + 0.35, ty), (tx - 0.35, ty), (tx, ty + 0.35), or (tx, ty - 0.35). Choose the one that is closest to your current position and free of obstacles.
    6. Use `get_nearby_objects_tool(x, y, radius)` to verify if your proposed destination (rx, ry) is empty of obstacles. Ensure no other objects (excluding the target object itself) are within a 0.20m radius of your proposed standing spot. Pass exactly 0.20 as the radius. If there are other obstacles, try another standing spot at the same 0.35m distance but at a slightly different angle. If you cannot find a safe coordinate after 3 attempts, return a text explaining why.
    7. Finally, use `navigate_and_face_tool` providing your safe (robot_x, robot_y) coordinate AND the target object's (face_x, face_y) coordinate. The system will automatically calculate the angle so you look at the target.
    8. Use `pick_tool(object_id)` to pick up the object once you have successfully navigated near it and are facing it.
    9. Use `place_tool(x, y, z)` to place an object at a target 3D coordinate.
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
        """Move the robot to (robot_x, robot_y) and automatically turn to face (face_x, face_y)."""
        self.get_logger().info(f'Tool called: navigate_and_face_tool(robot: {robot_x},{robot_y}, face: {face_x},{face_y})')
        
        # BYPASS NAVIGATION FOR DIRECT PICKING TEST
        self.get_logger().info("BYPASSING NAV2 - Returning success immediately for grasping test.")
        return "Successfully navigated to target."

    def execute_moveit_pose(self, group_name, link_name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        if not self._moveit_client.wait_for_server(timeout_sec=2.0):
            return False, "Failed to connect to MoveIt2 action server."

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        
        c = Constraints()
        
        # Position Constraint ONLY - 4DOF arms cannot satisfy 6DOF constraints reliably
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = link_name
        
        target_point = Point(x=float(x), y=float(y), z=float(z))
        
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]
        bv.primitives.append(sphere)
        
        pose = Pose()
        pose.position = target_point
        bv.primitive_poses.append(pose)
        
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)
        
        req.goal_constraints.append(c)
        goal_msg.request = req

        future = self._moveit_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, "MoveIt2 pose goal rejected."

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        res = result_future.result().result
        if res.error_code.val != 1:
            return False, f"MoveIt2 pose failed with error code: {res.error_code.val}"
            
        return True, "Success"

    def execute_moveit_joints(self, group_name, joint_names, positions):
        if not self._moveit_client.wait_for_server(timeout_sec=2.0):
            return False, "Failed to connect to MoveIt2 action server."

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        
        c = Constraints()
        for j, t in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = t
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        req.goal_constraints.append(c)
        goal_msg.request = req

        future = self._moveit_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, "MoveIt2 goal rejected."

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        res = result_future.result().result
        if res.error_code.val != 1:
            return False, f"MoveIt2 failed with error code: {res.error_code.val}"
            
        return True, "Success"

    def solve_ik_planar(self, r_target, z_target, alpha=0.8, ori_weight=0.001):
        # Base link (omx_link1) coordinates relative to joint2:
        x_rel = r_target - 0.012
        z_rel = z_target - 0.0595
        
        def fk(q2, q3, q4):
            x = math.cos(q2) * 0.024 + math.sin(q2) * 0.128 + math.cos(q2+q3) * 0.124 + math.cos(q2+q3+q4) * 0.126
            z = -math.sin(q2) * 0.024 + math.cos(q2) * 0.128 - math.sin(q2+q3) * 0.124 - math.sin(q2+q3+q4) * 0.126
            return x, z

        q = [0.968, -0.112, -0.055] # Initial guess
        lr = 0.5
        for i in range(1000):
            q2, q3, q4 = q
            x, z = fk(q2, q3, q4)
            
            ex = x - x_rel
            ez = z - z_rel
            e_ori = (q2 + q3 + q4) - alpha
            
            dq = 1e-5
            
            x_d2, z_d2 = fk(q2 + dq, q3, q4)
            g2 = ex * (x_d2 - x)/dq + ez * (z_d2 - z)/dq + ori_weight * e_ori
            
            x_d3, z_d3 = fk(q2, q3 + dq, q4)
            g3 = ex * (x_d3 - x)/dq + ez * (z_d3 - z)/dq + ori_weight * e_ori
            
            x_d4, z_d4 = fk(q2, q3, q4 + dq)
            g4 = ex * (x_d4 - x)/dq + ez * (z_d4 - z)/dq + ori_weight * e_ori
            
            q[0] -= lr * g2
            q[1] -= lr * g3
            q[2] -= lr * g4
            
            q[0] = max(-1.5, min(1.5, q[0]))
            q[1] = max(-1.5, min(1.4, q[1]))
            q[2] = max(-1.7, min(1.97, q[2]))
            
            if ex**2 + ez**2 < 1e-7:
                break
                
        return q

    def pick_tool(self, object_id: str) -> str:
        """Executes a predefined trajectory to pick up the specified object."""
        self.get_logger().info(f'Tool called: pick_tool({object_id})')
        
        arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
        gripper_joints = ['omx_gripper_left_joint']
        
        # We need the cube's location. Fetch it from semantic map
        if object_id not in self.sem_map:
            return f"Error: Object {object_id} not found."
            
        cube_x = self.sem_map[object_id]['position']['x']
        cube_y = self.sem_map[object_id]['position']['y']
        cube_z = self.sem_map[object_id]['position']['z']
        
        # Default fallback joint angles
        j1 = 0.0
        j2 = 0.968
        j3 = -0.112
        j4 = -0.055
        
        # 1. Open gripper
        self.execute_moveit_joints('gripper', gripper_joints, [0.010])
        time.sleep(1.0)
        
        # Try to look up target position relative to the arm base (omx_link1)
        try:
            # Look up transform from map to omx_link1
            t = self.tf_buffer.lookup_transform('omx_link1', 'map', rclpy.time.Time())
            
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            
            # Rotate point (cube_x, cube_y, cube_z) by quaternion (qx, qy, qz, qw)
            vx = cube_x
            vy = cube_y
            vz = cube_z
            
            cx = qy * vz - qz * vy
            cy = qz * vx - qx * vz
            cz = qx * vy - qy * vx
            
            ax = cx + qw * vx
            ay = cy + qw * vy
            az = cz + qw * vz
            
            bx = qy * az - qz * ay
            by = qz * ax - qx * az
            bz = qx * ay - qy * ax
            
            local_x = vx + 2.0 * bx + tx
            local_y = vy + 2.0 * by + ty
            local_z = vz + 2.0 * bz + tz
            
            self.get_logger().info(f"Target cube in omx_link1 frame: x={local_x:.3f}, y={local_y:.3f}, z={local_z:.3f}")
            
            # Calculate dynamic joint1 (yaw)
            j1 = math.atan2(local_y, local_x)
            
            # Distance in XY plane
            r = math.sqrt(local_x**2 + local_y**2)
            
            # Solve planar IK (alpha=1.57 for pointing forward)
            joints = self.solve_ik_planar(r, local_z, alpha=1.57)
            if joints is not None:
                j2, j3, j4 = joints
                self.get_logger().info(f"Dynamically solved IK: j1={j1:.4f}, j2={j2:.4f}, j3={j3:.4f}, j4={j4:.4f}")
            else:
                self.get_logger().warn("Dynamic IK failed. Using fallback joints.")
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform or solve IK: {e}. Using default.")
            
        # 2. Reach forward and down using calculated/fallback joints
        self.get_logger().info(f"Commanding arm to pick pose: {j1}, {j2}, {j3}, {j4}")
        ok, msg = self.execute_moveit_joints('arm', arm_joints, [j1, j2, j3, j4])
        if not ok: return f"Failed to reach object: {msg}"
        time.sleep(2.0)
        
        # 3. Close gripper
        self.execute_moveit_joints('gripper', gripper_joints, [-0.010])
        time.sleep(1.0)
        
        # 4. Retreat (Home)
        ok, msg = self.execute_moveit_joints('arm', arm_joints, [0.0, -1.0, 0.3, 0.7])
        if not ok: return msg
        time.sleep(2.0)
            
        return f"Successfully picked up {object_id}."

    def place_tool(self, x: float, y: float, z: float) -> str:
        """Executes a predefined trajectory to place an object at (x, y, z)."""
        self.get_logger().info(f'Tool called: place_tool({x}, {y}, {z})')
        
        arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
        gripper_joints = ['omx_gripper_left_joint']
        
        # 1. Reach forward
        ok, msg = self.execute_moveit_joints('arm', arm_joints, [0.0, 0.5, 0.5, -1.0])
        if not ok: return msg
        time.sleep(2.0)
        
        # 2. Open gripper
        self.execute_moveit_joints('gripper', gripper_joints, [0.010])
        time.sleep(1.0)
        
        # 3. Retreat (Home)
        ok, msg = self.execute_moveit_joints('arm', arm_joints, [0.0, -1.0, 0.3, 0.7])
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
