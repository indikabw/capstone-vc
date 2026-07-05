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

# --- Watchdog timeouts (seconds) ---
NAV_ACCEPT_TIMEOUT_SEC = 10.0
NAV_RESULT_TIMEOUT_SEC = 240.0
MOVEIT_ACCEPT_TIMEOUT_SEC = 10.0
MOVEIT_RESULT_TIMEOUT_SEC = 60.0
TASK_TIMEOUT_SEC = 600.0

# --- Deterministic visual servo tuning ---
CAMERA_WIDTH_PX = 1280
CAMERA_HEIGHT_PX = 720
CAMERA_HORIZONTAL_FOV_RAD = 1.25
CAMERA_FOCAL_PX = (CAMERA_WIDTH_PX / 2.0) / math.tan(CAMERA_HORIZONTAL_FOV_RAD / 2.0)
SERVO_MAX_ITERS = 8
SERVO_ALIGN_TOLERANCE_M = 0.02
SERVO_LATERAL_GAIN = 0.6
SERVO_DEPTH_GAIN = 0.6
SERVO_MAX_J1_STEP_RAD = 0.12
SERVO_MAX_J1_OFFSET_RAD = 0.4
SERVO_MAX_DEPTH_STEP_M = 0.03
SERVO_MIN_CONTOUR_AREA_PX = 40
SERVO_SETTLE_SEC = 1.5

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
        
        spatial_critic = Agent(
            name="spatial_critic",
            model="gemini-2.5-pro",
            description="Executes spatial reasoning and collision-aware robot trajectories. Use this to safely move the robot.",
            instruction="""
    You are the Spatial Critic Agent. The Semantic Planner will give you high-level intents.
    Your job is to ground these intents into safe, feasible robot movements.

    CRITICAL WORKFLOW for Picking Objects:
    Instead of executing a blind grasp, you MUST use the following sequence:
    1. If the incoming instruction explicitly states the robot is already positioned correctly and
       tells you not to navigate, skip straight to step 2. Otherwise, FIRST navigate to a safe pose
       near the object using `navigate_and_face_tool` before checking feasibility.
       - The arm is mounted well above the ground, so reaching a LOW grasp_z (near the ground) consumes most
         of the arm's 0.37m reach budget vertically, leaving very little horizontal budget. For any grasp_z
         below ~0.2m, stage the robot so its base is within ~0.25m of the object's (x, y) position - closer
         than you would for a normal "safe distance" stop. If feasibility keeps failing as 'out of reach' at
         a given distance, your next navigation attempt MUST move strictly closer, not to a similar or farther
         distance.
    2. Then, validate kinematics using `check_grasp_feasibility_tool(object_id, grasp_z, pitch_angle)`.
       - Try pitch_angle=1.57 (horizontal) or 3.14 (top-down).
       - If it returns 'out of reach', navigate strictly closer (see above) and retry, up to 3 times, before
         reporting failure.
    3. Once feasible, move to the hover position: call `hover_and_open_tool(object_id, grasp_z, pitch_angle)`.
    4. Call `visual_servo_align_tool(object_id)` exactly once. This runs a deterministic, code-only
       alignment loop (no further tool calls needed from you) and returns either:
       - "Alignment converged..." - proceed to step 5.
       - "Alignment failed: object not visible..." or "did not converge..." - navigate the base to a
         better vantage point (`navigate_and_face_tool`), then re-run from step 3.
    5. Call `adjust_pose_tool(dx=0.0, dy=0.0, dz=-0.1)` (or whatever descent is needed based on your hover z and target z) to lower the arm onto the object.
    6. Call `close_gripper_and_lift_tool()` to complete the pick!
    """,
            tools=[self.get_nearby_objects_tool, self.navigate_and_face_tool, self.check_grasp_feasibility_tool, self.hover_and_open_tool, self.visual_servo_align_tool, self.adjust_pose_tool, self.close_gripper_and_lift_tool, self.place_tool]
        )

        self.agent = Agent(
            name="robot_agent",
            model="gemini-2.5-pro",
            instruction="""
    You are an autonomous robot assistant controlling a TurtleBot4 (base radius 0.17m) with an OpenManipulator-X arm.
    You are the high-level Semantic Planner.
    
    CRITICAL INSTRUCTIONS:
    1. Use `list_objects_tool` to see all available objects in the environment. Look for keywords matching the user's request.
    2. If asked to go to a general room (e.g., 'kitchen', 'bedroom') or the 'center' of a room, deduce the area by finding objects that belong there. Calculate the center of the bounding box of these objects.
    3. Use `get_object_details_tool` to find the exact (x, y, z, yaw, size) of target objects. Use the object's own reported z position as the grasp_z you hand off - do not guess or reuse a value from a previous task.
    4. Formulate a step-by-step plan for the task.
    5. Delegate the actual execution of navigation and grasping to the `spatial_critic` sub-agent, passing the object's real (x, y, grasp_z) from step 3 (e.g., "Navigate to a safe spot near (x,y) and pick up '<object_id>' at grasp_z=<its actual reported z>").
    """,
            tools=[self.list_objects_tool, self.get_object_details_tool],
            sub_agents=[spatial_critic]
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
        
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return "Failed to connect to Nav2 action server."

        yaw = math.atan2(face_y - robot_y, face_x - robot_x)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(robot_x)
        goal_msg.pose.pose.position.y = float(robot_y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        future = self._nav_client.send_goal_async(goal_msg)
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > NAV_ACCEPT_TIMEOUT_SEC:
                return f"Nav2 did not accept the goal within {NAV_ACCEPT_TIMEOUT_SEC}s."
            time.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return "Nav2 goal rejected."

        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        while not result_future.done():
            if time.monotonic() - start > NAV_RESULT_TIMEOUT_SEC:
                goal_handle.cancel_goal_async()
                return f"Navigation timed out after {NAV_RESULT_TIMEOUT_SEC}s and was cancelled. Try a closer or simpler goal."
            time.sleep(0.1)

        res_obj = result_future.result()
        if res_obj.status == 4:
            return "Successfully navigated to target."
        else:
            return f"Navigation failed with status {res_obj.status}"

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
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > MOVEIT_ACCEPT_TIMEOUT_SEC:
                return False, f"MoveIt2 did not accept the goal within {MOVEIT_ACCEPT_TIMEOUT_SEC}s."
            time.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, "MoveIt2 pose goal rejected."

        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        while not result_future.done():
            if time.monotonic() - start > MOVEIT_RESULT_TIMEOUT_SEC:
                goal_handle.cancel_goal_async()
                return False, f"MoveIt2 pose timed out after {MOVEIT_RESULT_TIMEOUT_SEC}s and was cancelled."
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
            if 'gripper' in j:
                jc.tolerance_above = 0.001
                jc.tolerance_below = 0.001
            else:
                jc.tolerance_above = 0.05
                jc.tolerance_below = 0.05
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        req.goal_constraints.append(c)
        goal_msg.request = req

        future = self._moveit_client.send_goal_async(goal_msg)
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > MOVEIT_ACCEPT_TIMEOUT_SEC:
                return False, f"MoveIt2 did not accept the goal within {MOVEIT_ACCEPT_TIMEOUT_SEC}s."
            time.sleep(0.1)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, "MoveIt2 goal rejected."

        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        while not result_future.done():
            if time.monotonic() - start > MOVEIT_RESULT_TIMEOUT_SEC:
                goal_handle.cancel_goal_async()
                return False, f"MoveIt2 timed out after {MOVEIT_RESULT_TIMEOUT_SEC}s and was cancelled."
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

        guesses = [
            [0.968, -0.112, -0.055],
            [0.0, 0.0, 0.0],
            [-0.5, 1.0, -0.5],
            [0.5, 0.5, -1.0],
            [-0.5, 0.5, 0.0]
        ]
        
        best_q = None
        best_err = float('inf')
        
        for guess in guesses:
            q = list(guess)
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
                
                err = ex**2 + ez**2
                if err < 1e-7:
                    break
            
            err = ex**2 + ez**2
            if err < best_err:
                best_err = err
                best_q = list(q)
                
            if best_err < 1e-4:
                break
                
        if best_err > 1e-4:
            raise ValueError(f"Dynamic IK failed to converge to a reachable solution. Final error squared: {best_err:.6f}.")
                
        return best_q

    def calculate_ik_for_grasp(self, object_id: str, grasp_z: float, pitch_angle: float, r_offset: float = 0.0):
        """Helper function to calculate the IK solutions for grasping."""
        if object_id not in self.sem_map:
            raise ValueError(f"Object {object_id} not found.")
            
        cube_x = self.sem_map[object_id]['position']['x']
        cube_y = self.sem_map[object_id]['position']['y']
        cube_z = float(grasp_z) - 0.05
        
        t = self.tf_buffer.lookup_transform('omx_link1', 'map', rclpy.time.Time())
        
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        
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
        
        j1 = math.atan2(local_y, local_x)
        r = math.sqrt(local_x**2 + local_y**2)
        
        target_reach = math.sqrt((r - 0.03 + r_offset)**2 + local_z**2)
        if target_reach > 0.37:
            raise ValueError(f"Target is physically out of reach. Required reach is {target_reach:.2f}m, but max arm length is 0.37m. The object is either too far away or too high.")
        
        joints_pre = self.solve_ik_planar(r - 0.03 + r_offset, local_z, alpha=pitch_angle)
        joints_grasp = self.solve_ik_planar(r + 0.02 + r_offset, local_z, alpha=pitch_angle)
        joints_lift = self.solve_ik_planar(r + 0.02 + r_offset, local_z + 0.15, alpha=pitch_angle)
        
        return j1, joints_pre, joints_grasp, joints_lift

    def check_grasp_feasibility_tool(self, object_id: str, grasp_z: float, pitch_angle: float) -> str:
        """
        Validates if the robot can safely grasp the object at the specified grasp_z height and pitch_angle.
        Args:
            object_id: The name of the object to grasp.
            grasp_z: The absolute Z coordinate to grasp at.
            pitch_angle: The approach angle of the gripper in radians (0.0 is horizontal, -1.57 is top-down).
        """
        self.get_logger().info(f'Tool called: check_grasp_feasibility_tool({object_id}, z={grasp_z}, pitch={pitch_angle})')
        try:
            j1, j_pre, j_grasp, j_lift = self.calculate_ik_for_grasp(object_id, grasp_z, pitch_angle)
        except Exception as e:
            return f"Feasibility check failed: {e}"
            
        if j_pre is None or j_grasp is None or j_lift is None:
            return "Feasibility check failed: Dynamic IK failed to converge to a reachable solution."
            
        return "Feasibility check passed. The grasp is kinematically valid. You may now call hover_and_open_tool with these exact parameters."

    def hover_and_open_tool(self, object_id: str, grasp_z: float, pitch_angle: float) -> str:
        """Moves the arm to a hover position directly above the object (grasp_z + 0.15m) and opens the gripper."""
        self.get_logger().info(f'Tool called: hover_and_open_tool({object_id})')
        try:
            # We hover 0.15m above the intended grasp_z
            j1, j_pre, j_grasp, j_lift = self.calculate_ik_for_grasp(object_id, grasp_z + 0.15, pitch_angle)
            if j_grasp is None: return "Hover IK failed."
            
            # Store base joint1 and current grasp pose for visual_servo_align_tool / adjust_pose_tool
            self.current_grasp_params = {'object_id': object_id, 'z': grasp_z + 0.15, 'pitch': pitch_angle, 'r_offset': 0.0, 'j1': j1, 'j1_offset': 0.0}

            arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
            gripper_joints = ['omx_gripper_left_joint']

            self.execute_moveit_joints('gripper', gripper_joints, [0.019])
            time.sleep(1.0)

            ok, msg = self.execute_moveit_joints('arm', arm_joints, [j1] + list(j_grasp))
            if not ok: return f"Failed to hover: {msg}"

            return "Successfully moved to hover position and opened gripper. You can now call visual_servo_align_tool."
        except Exception as e:
            return f"Hover failed: {e}"

    def _find_object_pixel_bbox(self, img, object_id: str):
        """Segments the object in the image via HSV thresholding and returns (cx, cy, apparent_diameter_px) of the
        largest matching contour, or None if not found. Currently tuned for red objects only (the only color used
        in this scenario) - see docs/agentic_reasoning_evaluation.md for the color-generalization limitation."""
        import cv2
        import numpy as np
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower1, upper1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower2, upper2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < SERVO_MIN_CONTOUR_AREA_PX:
            return None
        x, y, w, h = cv2.boundingRect(largest)
        return (x + w / 2.0, y + h / 2.0, float(max(w, h)))

    def visual_servo_align_tool(self, object_id: str) -> str:
        """
        Deterministically aligns the gripper over the object using onboard camera feedback - no LLM calls
        inside the loop. Segments the object via HSV color thresholding, estimates its lateral pixel offset
        and apparent size, and drives a bounded proportional controller (fixed gain, clamped per-step
        correction, hard iteration cap) until the estimated lateral/depth error is below the 0.02m tolerance
        or the iteration budget is exhausted. Call this once after hover_and_open_tool; it runs the full
        alignment loop internally and returns only the final outcome.
        """
        self.get_logger().info(f'Tool called: visual_servo_align_tool({object_id})')
        if not hasattr(self, 'current_grasp_params'):
            return "Error: You must call hover_and_open_tool before aligning."

        obj = self.current_grasp_params['object_id']
        pitch = self.current_grasp_params['pitch']
        misses = 0
        reference_diameter_px = None

        for iteration in range(SERVO_MAX_ITERS):
            img = None
            with self.image_lock:
                if self.latest_image is not None:
                    img = self.latest_image.copy()

            if img is None:
                return "Alignment aborted: no camera image available."

            found = self._find_object_pixel_bbox(img, obj)
            if found is None:
                misses += 1
                if misses >= 2:
                    return ("Alignment failed: object not visible in the onboard camera after hover. "
                            "Consider navigating the base to a better vantage point and re-hovering.")
                time.sleep(SERVO_SETTLE_SEC)
                continue
            misses = 0

            px, py, apparent_diameter_px = found
            real_diameter_m = self.sem_map.get(obj, {}).get('size', {}).get('dx', 0.03) or 0.03

            depth_m = (real_diameter_m * CAMERA_FOCAL_PX) / apparent_diameter_px
            lateral_error_m = (px - CAMERA_WIDTH_PX / 2.0) * depth_m / CAMERA_FOCAL_PX
            self.last_measured_depth_m = depth_m

            if reference_diameter_px is None:
                reference_diameter_px = apparent_diameter_px
                depth_error_m = 0.0
            else:
                reference_depth_m = (real_diameter_m * CAMERA_FOCAL_PX) / reference_diameter_px
                depth_error_m = depth_m - reference_depth_m

            if abs(lateral_error_m) < SERVO_ALIGN_TOLERANCE_M and abs(depth_error_m) < SERVO_ALIGN_TOLERANCE_M:
                return (f"Alignment converged after {iteration + 1} iteration(s): "
                        f"lateral_error={lateral_error_m:.3f}m, depth_error={depth_error_m:.3f}m. "
                        f"You may now call adjust_pose_tool(dx=0, dy=0, dz=<descent>) to lower and grasp.")

            dtheta = max(-SERVO_MAX_J1_STEP_RAD, min(SERVO_MAX_J1_STEP_RAD, SERVO_LATERAL_GAIN * lateral_error_m / max(depth_m, 0.05)))
            depth_step = max(-SERVO_MAX_DEPTH_STEP_M, min(SERVO_MAX_DEPTH_STEP_M, SERVO_DEPTH_GAIN * depth_error_m))

            new_j1_offset = self.current_grasp_params['j1_offset'] + dtheta
            new_j1_offset = max(-SERVO_MAX_J1_OFFSET_RAD, min(SERVO_MAX_J1_OFFSET_RAD, new_j1_offset))
            new_r_offset = self.current_grasp_params['r_offset'] + depth_step

            self.current_grasp_params['j1_offset'] = new_j1_offset
            self.current_grasp_params['r_offset'] = new_r_offset

            try:
                j1_new, j_pre, j_grasp, j_lift = self.calculate_ik_for_grasp(
                    obj, self.current_grasp_params['z'], pitch, r_offset=new_r_offset)
                arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
                ok, msg = self.execute_moveit_joints('arm', arm_joints, [j1_new + new_j1_offset] + list(j_grasp))
                if not ok:
                    return f"Alignment step failed while correcting pose: {msg}"
            except Exception as e:
                return f"Alignment step error: {e}"

            time.sleep(SERVO_SETTLE_SEC)

        return (f"Alignment did not converge within {SERVO_MAX_ITERS} iterations "
                f"(last lateral_error={lateral_error_m:.3f}m, depth_error={depth_error_m:.3f}m). "
                f"Consider navigating the base to a better vantage point and retrying.")

    def adjust_pose_tool(self, dx: float, dy: float, dz: float) -> str:
        """Adjusts the arm position by the given dx (lateral), dy (depth/reach), and dz (height), in meters.
        Intended for the final controlled descent after visual_servo_align_tool reports convergence."""
        self.get_logger().info(f'Tool called: adjust_pose_tool(dx={dx}, dy={dy}, dz={dz})')
        if not hasattr(self, 'current_grasp_params'):
            return "Error: You must call hover_and_open_tool before adjusting."

        obj = self.current_grasp_params['object_id']
        current_z = self.current_grasp_params['z']
        pitch = self.current_grasp_params['pitch']
        current_r_offset = self.current_grasp_params['r_offset']
        current_j1_offset = self.current_grasp_params.get('j1_offset', 0.0)

        # Convert the requested lateral shift (meters) into a joint1 rotation using the last depth
        # estimate from visual_servo_align_tool, falling back to a conservative mid-range depth.
        depth_for_conversion = getattr(self, 'last_measured_depth_m', 0.2)
        dtheta = dx / max(depth_for_conversion, 0.05)
        new_j1_offset = max(-SERVO_MAX_J1_OFFSET_RAD, min(SERVO_MAX_J1_OFFSET_RAD, current_j1_offset + dtheta))

        new_r_offset = current_r_offset + dy
        new_z = current_z + dz

        self.current_grasp_params['z'] = new_z
        self.current_grasp_params['r_offset'] = new_r_offset
        self.current_grasp_params['j1_offset'] = new_j1_offset

        try:
            j1_new, j_pre, j_grasp, j_lift = self.calculate_ik_for_grasp(obj, new_z, pitch, r_offset=new_r_offset)

            arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
            ok, msg = self.execute_moveit_joints('arm', arm_joints, [j1_new + new_j1_offset] + list(j_grasp))
            if not ok: return f"Adjustment failed: {msg}"

            return f"Successfully adjusted pose. New Z is {new_z}."
        except Exception as e:
            return f"Adjustment error: {e}"

    def close_gripper_and_lift_tool(self) -> str:
        """Closes the gripper to grasp the object and lifts the arm back to the home position."""
        self.get_logger().info('Tool called: close_gripper_and_lift_tool()')
        if not hasattr(self, 'current_grasp_params'):
            return "Error: No grasp context. Did you hover?"
            
        gripper_joints = ['omx_gripper_left_joint']
        arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
        
        self.execute_moveit_joints('gripper', gripper_joints, [-0.008])
        time.sleep(1.0)
        
        try:
            # Lift
            j1_new, j_pre, j_grasp, j_lift = self.calculate_ik_for_grasp(
                self.current_grasp_params['object_id'],
                self.current_grasp_params['z'] + 0.15,
                self.current_grasp_params['pitch'],
                r_offset=self.current_grasp_params['r_offset']
            )
            j1_offset = self.current_grasp_params.get('j1_offset', 0.0)
            self.execute_moveit_joints('arm', arm_joints, [j1_new + j1_offset] + list(j_lift))
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Lift IK failed: {e}")
        
        # Retreat to Home
        self.execute_moveit_joints('arm', arm_joints, [0.0, -1.0, 0.3, 0.7])
        time.sleep(2.0)
        return "Successfully gripped and lifted the object."


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

            async def run_adk_with_deadline():
                return await asyncio.wait_for(run_adk(), timeout=TASK_TIMEOUT_SEC)

            # Run the ADK asyncio loop safely in this worker thread, bounded by an overall task deadline
            response = asyncio.run(run_adk_with_deadline())
            summary = str(response)
            task_failed = False
        except asyncio.TimeoutError:
            self.get_logger().error(f"ADK Agent exceeded the {TASK_TIMEOUT_SEC}s task deadline.")
            summary = f"Reasoning failed: task exceeded {TASK_TIMEOUT_SEC}s deadline."
            task_failed = True
        except Exception as e:
            self.get_logger().error(f"ADK Agent failed: {e}")
            summary = f"Reasoning failed: {e}"
            task_failed = True

        self.get_logger().info('Reasoning complete.')

        result = ReasoningTask.Result()
        result.success = not task_failed
        result.summary = summary
        if task_failed:
            goal_handle.abort()
        else:
            goal_handle.succeed()
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
