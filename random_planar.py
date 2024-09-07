import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import math

class PlanarRandomMotion(Node):

    def __init__(self):
        super().__init__('planar_random_motion')
        self.publisher_ = self.create_publisher(Twist, '/planar/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz (1 s)

    def timer_callback(self):
        msg = Twist()
        # Set linear velocities to zero (not moving linearly)
        msg.linear.x = random.uniform(-1.0, 1.0)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # Set random angular velocity around z-axis (yaw)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = random.uniform(-1.0, 1.0)  # Random angular velocity between -1 and 1 rad/s

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published random angular velocity: {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    planar_random_motion = PlanarRandomMotion()
    rclpy.spin(planar_random_motion)
    planar_random_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

