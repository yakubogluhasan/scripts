import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from rclpy.qos import qos_profile_sensor_data
from tello_msgs.msg import Displacement, FlightData
from std_msgs.msg import Float64


class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('drone_imu_listener')
        self.subscription = self.create_subscription(FlightData, '/flight_data', self.callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Displacement, '/displacement', 10)
        self.get_logger().info('IMUSubscriber has been started')
        #self.publisher_ = self.create_publisher(Float64, '/rotation_publisher', 10)
        self.initial_x_velocity = 0.0
        self.initial_y_velocity = 0.0
        self.initial_z_velocity = 0.0        
        self.x_displacement = 0.0
        self.y_displacement = 0.0
        self.z_displacement = 0.0
    def callback(self, msg):
        
        self.velocity = [msg.vgx, msg.vgy, msg.vgz]
        drone_displacement = Displacement()
        x_velocity = self.velocity[0]
        y_velocity = self.velocity[1]
        z_velocity = self.velocity[2]
        
        
         #quat_list = [msg.roll, msg.pitch, msg.yaw]
        
#        x_velocity = self.initial_x_velocity + self.acceleration[0] * 0.1
#        y_velocity = self.initial_y_velocity + self.acceleration[1] * 0.1
#        rx_displacement = self.old_x_velocity * 0.1 + 0.5 * self.acceleration[0] * 0.1**2
#        ry_displacement = self.old_y_velocity * 0.1 + 0.5 * self.acceleration[1] * 0.1**2
        
        rx_displacement = 0.5 * (self.initial_x_velocity + x_velocity) * 0.1
        ry_displacement = 0.5 * (self.initial_y_velocity + y_velocity) * 0.1
        rz_displacement = 0.5 * (self.initial_z_velocity + z_velocity) * 0.1
        
        
        self.x_displacement += rx_displacement
        self.y_displacement += ry_displacement
        self.z_displacement += rz_displacement
        
        self.initial_x_velocity = x_velocity
        self.initial_y_velocity = y_velocity
        self.initial_z_velocity = z_velocity
        #self.old_yaw = yaw
        
        drone_displacement.rx = self.x_displacement
        drone_displacement.ry = self.y_displacement
        drone_displacement.rz = self.y_displacement
        self.publisher.publish(drone_displacement)
        self.get_logger().info(f"{self.x_displacement}, {self.y_displacement}, {self.z_displacement}")
            
def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

