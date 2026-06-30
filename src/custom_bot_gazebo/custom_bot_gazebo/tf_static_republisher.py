import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class TfStaticRepublisher(Node):
    def __init__(self):
        super().__init__('tf_static_republisher')
        
        # Subscribe to /tf_static to get all static transforms
        qos_static = QoSProfile(depth=100, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(TFMessage, '/tf_static', self.tf_static_cb, qos_static)
        
        # Publish them as dynamic transforms on /tf
        self.pub = self.create_publisher(TFMessage, '/tf', 10)
        
        self.static_transforms = {}
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("TF Static Republisher started!")

    def tf_static_cb(self, msg):
        # Store latest static transforms
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            self.static_transforms[key] = t

    def timer_cb(self):
        if not self.static_transforms:
            return
            
        msg = TFMessage()
        now = self.get_clock().now().to_msg()
        for t in self.static_transforms.values():
            t_dyn = t
            t_dyn.header.stamp = now
            msg.transforms.append(t_dyn)
            
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TfStaticRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
