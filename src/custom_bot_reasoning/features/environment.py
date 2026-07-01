import rclpy
import threading
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from mocks.mock_nav2_server import MockNav2Server
from mocks.mock_moveit_server import MockMoveItServer

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from custom_bot_reasoning.reasoning_node import ReasoningNode

def before_all(context):
    rclpy.init()
    context.mock_nav2_server = MockNav2Server()
    context.mock_moveit_server = MockMoveItServer()
    context.reasoning_node = ReasoningNode()
    
    context.executor = rclpy.executors.MultiThreadedExecutor()
    context.executor.add_node(context.mock_nav2_server)
    context.executor.add_node(context.mock_moveit_server)
    context.executor.add_node(context.reasoning_node)
    context.spin_thread = threading.Thread(target=context.executor.spin, daemon=True)
    context.spin_thread.start()

def after_all(context):
    context.executor.shutdown()
    rclpy.shutdown()
    context.spin_thread.join(timeout=1.0)
