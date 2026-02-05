import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import DurabilityPolicy

class ObjectChaser(Node):
    def __init__(self):
        super().__init__('object_chaser')
        
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos_policy = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_policy)
            
        cmd_vel_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
          

        self.publsihTwist = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            cmd_vel_qos
        )
        
        print("Object Chaser Started! Mencari benda terdekat...")

    def scan_callback(self, msg):
        twist = Twist()
        ranges = np.array(msg.ranges)
        for i in range(len(ranges)):
            if ranges[i] == float('inf') or ranges[i] == 0.0:
                ranges[i] = 50.0
            
        for i in range(len(ranges)):
            if ranges[i] > 2.0 and ranges[i] < 30.0:
                twist.linear.x = 1.0 
                
        self.publsihTwist.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectChaser()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()