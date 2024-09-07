import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from rclpy.qos import qos_profile_sensor_data
from tello_msgs.msg import Displacement
from std_msgs.msg import Float64
from tello_msgs.msg import FlightData

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('drone_imu_listener')
        self.subscription = self.create_subscription(FlightData, '/flight_data', self.callback, qos_profile_sensor_data)
        self.get_logger().info('IMUSubscriber has been started')
        self.publisher = self.create_publisher(Imu, '/imu_data', 10)
        self.imu_data = Imu()
        
    def callback(self, msg):
        
        rot = Rotation.from_euler('xyz', [msg.roll, msg.pitch, msg.yaw])
        [x, y, z, w] = rot.as_quat()
        self.imu_data.orientation.x = x
        self.imu_data.orientation.y = y
        self.imu_data.orientation.z = z
        self.imu_data.orientation.w = w
        self.publisher.publish(self.imu_data)
        #rot = Rotation.from_quat(quat_list)
        #[roll, pitch, yaw] = rot.as_euler('xyz', degrees=True)
        
        #displacement = Displacement()
        #acceleration = Float64()
        
        #acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        
        #x_velocity = acceleration[0] * 0.1
        #y_velocity = acceleration[1] * 0.1
        #z_velocity = acceleration[2] * 0.1
        
#        x_displacement = 100 * 0.5 * acceleration[0] * 0.1**2
#        y_displacement = 100 * 0.5 * acceleration[1] * 0.1**2
#        z_displacement = 100 * 0.5 * acceleration[2] * 0.1**2
        
#        displacement.rx = x_displacement
#        displacement.ry = y_displacement
#        displacement.rz = z_displacement
#        displacement.total = math.sqrt(x_displacement**2 + y_displacement**2 + z_displacement**2)
        
        
        #self.publisher.publish(displacement)
        #self.get_logger().info(f"Vx: {x_velocity}, Vy: {y_velocity}, Vz: {z_velocity}")
            
def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

