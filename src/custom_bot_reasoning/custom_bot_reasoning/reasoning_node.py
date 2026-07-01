import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from custom_bot_interfaces.action import ReasoningTask
from cv_bridge import CvBridge
import threading
import time

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
        
        self.heartbeat_timer = self.create_timer(
            0.1,
            self.heartbeat_callback,
            callback_group=self.callback_group
        )
        
    def heartbeat_callback(self):
        pass
        
    def image_callback(self, msg):
        with self.image_lock:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Failed to convert image: {e}')

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
            self.get_logger().warn('No image received yet on /camera/image_raw')
        else:
            self.get_logger().info('Successfully sampled image frame for reasoning.')
            
        feedback_msg.current_stage = "reasoning"
        goal_handle.publish_feedback(feedback_msg)
        
        # Simulate ADK blocking call
        time.sleep(1.0)
        
        self.get_logger().info('Reasoning complete.')
        goal_handle.succeed()
        
        result = ReasoningTask.Result()
        result.success = True
        result.summary = f'Successfully processed command: {goal_handle.request.command}'
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
