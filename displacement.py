import rclpy
import math
from rclpy.node import Node
#from geometry_msgs.msg import Twist
#from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
#from std_msgs.msg import Float32 
from rclpy.qos import qos_profile_sensor_data
from tello_msgs.msg import Displacement
from sensor_msgs.msg import Imu

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.subscription = self.create_subscription(
            Imu,
            '/tb3/imu',
            self.ImuCallback,
            qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Odometry,
            '/planar/odom',
            self.OdomCallback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Displacement, '/planar_displacement', 10)
        self.x_displacement = 0.0
        self.y_displacement = 0.0
        self.initial_x_velocity = 0.0
        self.initial_y_velocity = 0.0
        self.initial_yaw = 0.0
        self.velocity = [0.0, 0.0]
        self.subscription  # prevent unused variable warning
        
    def OdomCallback(self, msg):
        self.velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y] 
        
            
    def ImuCallback(self, msg):
        planar = Displacement()
#        x_velocity = 0.0
#        y_velocity = 0.0
        x_velocity = self.velocity[0]
        y_velocity = self.velocity[1]
        
        self.acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y]
        quat_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot = Rotation.from_quat(quat_list)
        [roll, pitch, yaw] = rot.as_euler('xyz', degrees=True)
        
#        x_velocity = self.initial_x_velocity + self.acceleration[0] * 0.1
#        y_velocity = self.initial_y_velocity + self.acceleration[1] * 0.1
        
        rx_displacement = 0.5 * (self.initial_x_velocity + x_velocity) * 0.05
        ry_displacement = 0.5 * (self.initial_y_velocity + y_velocity) * 0.05
#        rx_displacement = self.old_x_velocity * 0.1 + 0.5 * self.acceleration[0] * 0.1**2
#        ry_displacement = self.old_y_velocity * 0.1 + 0.5 * self.acceleration[1] * 0.1**2
        
        self.x_displacement += rx_displacement
        self.y_displacement += ry_displacement
        
        self.initial_x_velocity = x_velocity
        self.initial_y_velocity = y_velocity
        self.old_yaw = yaw
        
        planar.rx = self.x_displacement
        planar.ry = self.y_displacement
        
        self.publisher.publish(planar)
        
        self.get_logger().info(f"old x: {self.initial_x_velocity} x: {x_velocity}")
        self.get_logger().info(f"x: {self.x_displacement} y: {self.y_displacement}")
        

            
def main(args=None):
    rclpy.init(args=args)
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

