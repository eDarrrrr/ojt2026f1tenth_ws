import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class OdomTFBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')

        self.broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            TFMessage,
            '/f1tenth_controller/tf_odometry',
            self.callback,
            10
        )

    def callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdomTFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
