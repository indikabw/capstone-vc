import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, JointState
from custom_bot_interfaces.action import ReasoningTask
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import Point, Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
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
        self.latest_joint_states = {}
        self.joint_lock = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10,
            callback_group=self.callback_group
        )

        # Direct gripper command publisher. MoveIt-planned gripper moves execute a trajectory but do not
        # reliably actuate the fingers in this Gazebo setup (the gripper joint never appears in /joint_states
        # and grasps never make contact), so the gripper is commanded straight to its controller instead.
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
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
    You are the Spatial Critic Agent. The Semantic Planner gives you a high-level intent; you ground it
    into a safe, validated grasp. Reason about the grasp STRATEGY once, then execute a single atomic grasp.
    Do NOT search grasp heights or approach angles by trial and error, and do NOT run any per-step visual
    alignment loop - the object pose is already known and the grasp height is computed for you.

    GRASP STRATEGY:
    - The grasp is performed TOP-DOWN: the gripper points straight down, descends over the object's center so
      its open jaws pass around the body, then closes and lifts. This is the approach this arm reaches most
      reliably. The grasp height and approach are computed for you; pass pitch_angle=1.57.

    WORKFLOW (follow in order, once each unless a step tells you to retry):
    1. Positioning: if the incoming instruction says the robot is already positioned correctly and to not
       navigate, skip to step 2. Otherwise call `navigate_and_face_tool` to stand ~0.30-0.35m from the
       object, facing it.
    2. VETO CHECK: call `check_grasp_feasibility_tool(object_id, pitch_angle=1.57)`. If it reports the object
       is out of reach, move strictly closer with `navigate_and_face_tool` and retry (at most 3 times). If it
       still fails, report honest failure and stop.
    3. Once feasibility passes, call `execute_grasp_tool(object_id, pitch_angle=1.57)` exactly once. This runs
       the entire atomic grasp (open, hover, descend, close, lift, retreat). No other arm tools are needed.
    4. Call `verify_grasp_tool(object_id)` to visually confirm the object is held and lifted. Report SUCCESS
       only if verification is POSITIVE. If it is NEGATIVE or inconclusive, report honest failure - never
       claim success the camera does not confirm.
    """,
            tools=[self.get_nearby_objects_tool, self.navigate_and_face_tool, self.check_grasp_feasibility_tool, self.execute_grasp_tool, self.verify_grasp_tool, self.place_tool]
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
    3. Use `get_object_details_tool` to find the exact (x, y, z, yaw, size) of target objects.
    4. Formulate a step-by-step plan for the task.
    5. Unless the user's instruction says the robot is already positioned and to not navigate, call
       `navigate_to_standoff_tool(object_id)` to approach the target object. It computes the standing
       position and orientation itself - you do not choose coordinates.
    6. Delegate execution to the `spatial_critic` sub-agent by object_id (e.g., "Pick up '<object_id>'."). The
       critic computes the grasp height and approach itself - you do not need to specify grasp_z or angles.
    """,
            tools=[self.list_objects_tool, self.get_object_details_tool, self.navigate_to_standoff_tool],
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

    def joint_states_callback(self, msg):
        with self.joint_lock:
            for name, pos in zip(msg.name, msg.position):
                self.latest_joint_states[name] = pos

    def _gripper_position(self):
        """Latest commanded/measured position of the gripper joint, or None if not yet received."""
        with self.joint_lock:
            return self.latest_joint_states.get('omx_gripper_left_joint')

    def control_gripper(self, position, settle=1.5):
        """Command the gripper directly to its controller (bypassing MoveIt), which reliably actuates the
        fingers in this Gazebo setup. position: joint value (0.019 open ... -0.011 closed)."""
        msg = JointTrajectory()
        msg.joint_names = ['omx_gripper_left_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(pt)
        self.gripper_pub.publish(msg)
        time.sleep(settle)

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

    # Distance from the object center to stand at before grasping. Computed in code (not chosen by the
    # LLM) because it has to satisfy two opposing constraints at once: far enough that the object (and its
    # stand) fall outside the local costmap's inflation radius (~0.17m robot_radius + 0.18m inflation), but
    # close enough that the object stays within the arm's ~0.42m usable reach once stopped. 0.34m is the
    # verified distance from the direct (no-nav) grasp tests.
    GRASP_STANDOFF_M = 0.34

    def navigate_to_standoff_tool(self, object_id: str) -> str:
        """Navigates the robot to a standoff position near the target object and faces it, so a grasp can
        be attempted afterwards. The standoff distance and approach direction are computed in code from the
        robot's current pose and the object's position - the LLM does not choose the coordinates. Call this
        before delegating to the spatial_critic, unless the instructions say the robot is already positioned."""
        self.get_logger().info(f'Tool called: navigate_to_standoff_tool({object_id})')
        if object_id not in self.sem_map:
            return f"Object {object_id} not found."
        ox = self.sem_map[object_id]['position']['x']
        oy = self.sem_map[object_id]['position']['y']

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            return f"Failed to get robot pose: {e}"
        rx, ry = t.transform.translation.x, t.transform.translation.y

        dx, dy = rx - ox, ry - oy
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            dx, dy, dist = 1.0, 0.0, 1.0
        ux, uy = dx / dist, dy / dist

        target_x = ox + ux * self.GRASP_STANDOFF_M
        target_y = oy + uy * self.GRASP_STANDOFF_M
        return self.navigate_and_face_tool(target_x, target_y, ox, oy)

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

        # Diagnostic: the planar radius, height, and straight-line reach to the grasp point, in the arm base
        # frame. Logged so the actual required reach can be compared against the arm's usable envelope.
        target_reach = math.sqrt((r - 0.03 + r_offset)**2 + local_z**2)
        self.get_logger().info(f'IK geometry for {object_id}: r={r:.3f} local_z={local_z:.3f} target_reach={target_reach:.3f}')
        # The hard cap was previously 0.37m, which rejected the floor grasp that commit d1c40a0 physically
        # performed (the cap was added in d2debd8, after that verified pickup). Relax to the arm's true usable
        # reach and let the IK solver's own convergence check be the real feasibility gate.
        if target_reach > 0.42:
            raise ValueError(f"Target is physically out of reach. Required reach is {target_reach:.2f}m, beyond the arm's usable envelope (~0.42m). Move the base closer to the object.")
        
        joints_pre = self.solve_ik_planar(r - 0.03 + r_offset, local_z, alpha=pitch_angle)
        joints_grasp = self.solve_ik_planar(r + 0.02 + r_offset, local_z, alpha=pitch_angle)
        joints_lift = self.solve_ik_planar(r + 0.02 + r_offset, local_z + 0.15, alpha=pitch_angle)

        return j1, joints_pre, joints_grasp, joints_lift

    def _map_point_to_arm_frame(self, mx, my, mz):
        """Express a map-frame point (mx,my,mz) in the arm base (omx_link1) frame, returning (j1, r, local_z)
        where j1 is the base-rotation azimuth to face it, r the planar radius, and local_z its height."""
        t = self.tf_buffer.lookup_transform('omx_link1', 'map', rclpy.time.Time())
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
        vx, vy, vz = mx, my, mz
        cx = qy * vz - qz * vy; cy = qz * vx - qx * vz; cz = qx * vy - qy * vx
        ax = cx + qw * vx; ay = cy + qw * vy; az = cz + qw * vz
        bx = qy * az - qz * ay; by = qz * ax - qx * az; bz = qx * ay - qy * ax
        local_x = vx + 2.0 * bx + tx
        local_y = vy + 2.0 * by + ty
        local_z = vz + 2.0 * bz + tz
        return math.atan2(local_y, local_x), math.sqrt(local_x**2 + local_y**2), local_z

    # Correction added to the object-center world z when choosing the IK target height for a top-down grasp,
    # so the gripper FINGERTIPS (not the wrist) end up at the object body. Calibrated from a measured run:
    # targeting center+0.08 left the fingertip 0.126m too high, so the net target must be ~0.13m lower.
    GRIPPER_FINGER_OFFSET = -0.05

    # Radial correction: measured runs showed the gripper center landing ~0.02m beyond the object (overshoot
    # in reach), so the jaws closed just past it. Pull the planar radius in by this much to center the jaws.
    GRIPPER_RADIAL_OFFSET = -0.02

    def calculate_topdown_grasp(self, object_id: str):
        """Top-down grasp poses: the gripper points straight down and descends vertically over the object's
        center (no radial push that would knock a thin object over). Returns (j1, hover, grip, lift) joint sets.
        The grip level places the fingertips around the object's center height."""
        if object_id not in self.sem_map:
            raise ValueError(f"Object {object_id} not found.")
        cx = self.sem_map[object_id]['position']['x']
        cy = self.sem_map[object_id]['position']['y']
        center_z = float(self.sem_map[object_id]['position']['z'])

        # IK-target (link end) heights, in the arm frame, for each phase. The fingertips sit
        # GRIPPER_FINGER_OFFSET below the link end, so to put the fingertips at the object center the link end
        # must be that much higher.
        j1, r, lz_center = self._map_point_to_arm_frame(cx, cy, center_z)
        _, _, lz_grip = self._map_point_to_arm_frame(cx, cy, center_z + self.GRIPPER_FINGER_OFFSET)
        _, _, lz_hover = self._map_point_to_arm_frame(cx, cy, center_z + self.GRIPPER_FINGER_OFFSET + 0.12)
        _, _, lz_lift = self._map_point_to_arm_frame(cx, cy, center_z + self.GRIPPER_FINGER_OFFSET + 0.18)

        reach = math.sqrt(r**2 + lz_grip**2)
        self.get_logger().info(f'Top-down IK for {object_id}: r={r:.3f} lz_grip={lz_grip:.3f} reach={reach:.3f}')
        if reach > 0.42:
            raise ValueError(f"Top-down target out of reach ({reach:.2f}m). Move the base closer.")

        # alpha=1.57 -> gripper points straight down; radius pulled in by the measured overshoot to center jaws.
        r_target = r + self.GRIPPER_RADIAL_OFFSET
        self.get_logger().info(f'Top-down grasp r_target={r_target:.3f} (raw r={r:.3f}, radial_offset={self.GRIPPER_RADIAL_OFFSET})')
        hover = self.solve_ik_planar(r_target, lz_hover, alpha=1.57)
        grip = self.solve_ik_planar(r_target, lz_grip, alpha=1.57)
        lift = self.solve_ik_planar(r_target, lz_lift, alpha=1.57)
        return j1, hover, grip, lift

    def _grasp_z_for(self, object_id: str) -> float:
        """The grasp height is the object's own center z from the semantic map. It is computed in code,
        never chosen by the LLM, so the gripper always closes around the object's body (not its top edge)."""
        if object_id not in self.sem_map:
            raise ValueError(f"Object {object_id} not found in the semantic map.")
        return float(self.sem_map[object_id]['position']['z'])

    def check_grasp_feasibility_tool(self, object_id: str, pitch_angle: float) -> str:
        """
        Veto check: validates that the object is kinematically reachable for a top-down grasp before any
        motion. The grasp height is computed automatically from the object's center - you do not choose it.
        Args:
            object_id: The name of the object to grasp.
            pitch_angle: Kept for interface compatibility; the grasp is performed top-down regardless.
        """
        self.get_logger().info(f'Tool called: check_grasp_feasibility_tool({object_id})')
        try:
            j1, hover, grip, lift = self.calculate_topdown_grasp(object_id)
        except Exception as e:
            return f"Feasibility check failed: {e}"
        if hover is None or grip is None or lift is None:
            return "Feasibility check failed: IK did not converge to a reachable top-down solution."
        return "Feasibility check passed. The top-down grasp is kinematically valid. Call execute_grasp_tool now."

    def execute_grasp_tool(self, object_id: str, pitch_angle: float) -> str:
        """
        Executes the full atomic grasp as one validated sequence - no camera in the loop, no per-step
        perturbation. The gripper points straight down and descends vertically over the object's center so
        the open jaws pass around the body, then closes and lifts straight up (a top-down grasp - it does not
        push sideways, which would knock a thin object over). The grasp height is computed in code. Call this
        once, after check_grasp_feasibility_tool passes; then call verify_grasp_tool to confirm the result.
        Args:
            object_id: The name of the object to grasp.
            pitch_angle: Kept for interface compatibility; the grasp is performed top-down regardless.
        """
        arm_joints = ['omx_joint1', 'omx_joint2', 'omx_joint3', 'omx_joint4']
        gripper_joints = ['omx_gripper_left_joint']
        self.get_logger().info(f'Tool called: execute_grasp_tool({object_id})')

        try:
            j1, hover, grip, lift = self.calculate_topdown_grasp(object_id)
        except Exception as e:
            return f"Execution failed: {e}"
        if hover is None or grip is None or lift is None:
            return "Execution failed: IK did not converge."

        # 1. Open the gripper fully (direct controller command)
        self.control_gripper(0.019)

        # 2. Hover directly above the object center, gripper pointing down
        self.get_logger().info("Moving to hover pose above object")
        ok, msg = self.execute_moveit_joints('arm', arm_joints, [j1] + list(hover))
        if not ok:
            return f"Failed to reach hover: {msg}"
        time.sleep(1.0)

        # 3. Descend vertically so the open jaws pass around the object body
        self.get_logger().info("Descending onto object")
        ok, msg = self.execute_moveit_joints('arm', arm_joints, [j1] + list(grip))
        if not ok:
            return f"Failed to descend: {msg}"
        time.sleep(1.0)

        # Diagnostic: measure where the gripper fingertip actually is in map frame vs the object, so any
        # residual grasp offset can be read directly and corrected instead of guessed.
        try:
            op = self.sem_map[object_id]['position']
            for link in ('omx_gripper_left_link', 'omx_link5'):
                gt = self.tf_buffer.lookup_transform('map', link, rclpy.time.Time())
                gx, gy, gz = gt.transform.translation.x, gt.transform.translation.y, gt.transform.translation.z
                self.get_logger().info(
                    f'GRIP-OFFSET {link}: gripper=({gx:.3f},{gy:.3f},{gz:.3f}) object=({op["x"]:.3f},{op["y"]:.3f},{op["z"]:.3f}) '
                    f'dx={op["x"]-gx:+.3f} dy={op["y"]-gy:+.3f} dz={op["z"]-gz:+.3f}')
        except Exception as e:
            self.get_logger().warn(f'Grip-offset TF lookup failed: {e}')

        # 4. Close the gripper around the object body (direct controller command; full close for a firm hold)
        self.get_logger().info("Closing gripper")
        self.control_gripper(-0.011, settle=2.0)

        # Code-level grip check: if the gripper settled well above its fully-closed limit (-0.011), it is
        # blocked by the object (holding something); if it reached the limit it closed on empty air.
        grip_pos = self._gripper_position()
        grip_note = ""
        if grip_pos is not None:
            if grip_pos > -0.0105:
                grip_note = f" Gripper settled at {grip_pos:.4f} (blocked by object -> likely holding it)."
            else:
                grip_note = f" WARNING: gripper closed to {grip_pos:.4f} (empty air -> likely missed the object)."

        # 5. Lift straight up
        self.get_logger().info("Lifting object")
        ok, msg = self.execute_moveit_joints('arm', arm_joints, [j1] + list(lift))
        if not ok:
            return f"Failed to lift: {msg}.{grip_note}"
        time.sleep(1.0)

        # 6. Retreat to home with the object
        self.execute_moveit_joints('arm', arm_joints, [0.0, -1.0, 0.3, 0.7])
        time.sleep(2.0)
        return (f"Grasp sequence executed for {object_id}.{grip_note} "
                f"Now call verify_grasp_tool({object_id}) to visually confirm the object is held and lifted.")

    def verify_grasp_tool(self, object_id: str) -> str:
        """
        Vision-only verification (no metric estimation): samples the onboard camera and asks the vision model
        a yes/no question about whether the object is currently held in the gripper and lifted off its surface.
        Use this after execute_grasp_tool. Only report task success if this returns held=true.
        """
        self.get_logger().info(f'Tool called: verify_grasp_tool({object_id})')
        img = None
        with self.image_lock:
            if self.latest_image is not None:
                img = self.latest_image.copy()
        if img is None:
            return "Verification inconclusive: no camera image available. Do not claim success."

        import cv2
        _, buffer = cv2.imencode('.jpg', img)
        image_bytes = buffer.tobytes()
        try:
            from google import genai
            from google.genai import types
            client = genai.Client()
            prompt = (
                f"This is a robot's onboard camera view immediately after a pick-up attempt of '{object_id}'. "
                f"Is the object currently held in the robot's gripper and lifted off its resting surface? "
                f"Respond with strictly a JSON object and nothing else: "
                f"{{\"held\": true or false, \"reason\": \"one short phrase\"}}."
            )
            resp = client.models.generate_content(
                model='gemini-2.5-pro',
                contents=[types.Part.from_bytes(data=image_bytes, mime_type='image/jpeg'), prompt]
            )
            text = resp.text.strip().replace('```json', '').replace('```', '')
            parsed = json.loads(text)
            held = bool(parsed.get("held", False))
            reason = str(parsed.get("reason", ""))
            if held:
                return f"Verification POSITIVE: the object appears held and lifted ({reason}). You may report success."
            return f"Verification NEGATIVE: the object does not appear held/lifted ({reason}). Report honest failure - do not claim success."
        except Exception as e:
            self.get_logger().error(f"Vision verification failed: {e}")
            return "Verification inconclusive: the vision model could not be reached. Do not claim success."


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
