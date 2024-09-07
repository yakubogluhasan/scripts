import rclpy
import math
from rclpy.node import Node
from tello_msgs.msg import FlightData, Displacement
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Int32

class Data(Node):
    def __init__(self):
        super().__init__('acceleration')
        self.subscription = self.create_subscription(FlightData, '/flight_data', self.callback, qos_profile_sensor_data)
        self.get_logger().info('displacement node has been started')
        self.publisher = self.create_publisher(Displacement, '/displacement_publisher', 10)
        self.file = open("displacement_data.txt", "w")  # Metin dosyasını yazmak için aç
        self.x_displacement = 0.0
        self.y_displacement = 0.0
        self.z_displacement = 0.0
        self.path = 0.0
    def callback(self, msg):
#        acceleration = Float32MultiArray()
        displacement = Displacement()
        
#        rpy = Int32()
#        rpy = [msg.roll, msg.pitch, msg.yaw]
        
        velocity = [msg.vgx, msg.vgy, msg.vgz]
        
        rx_displacement = velocity[0] * 0.1
        ry_displacement = velocity[1] * 0.1 
        rz_displacement = velocity[2] * 0.1 

        self.x_displacement += rx_displacement
        self.y_displacement += ry_displacement
        self.z_displacement += rz_displacement
        
        displacement.rx = rx_displacement
        displacement.ry = ry_displacement
        displacement.rz = rz_displacement
        self.path += math.sqrt(rx_displacement**2 + ry_displacement**2 + rz_displacement**2)
#        displacement.total = math.sqrt(rx_displacement**2 + ry_displacement**2 + rz_displacement**2)
        displacement.total = math.sqrt(self.x_displacement**2 + self.y_displacement**2 + self.z_displacement**2)
        
        # Verileri dosyaya yaz
        self.file.write(f"{rx_displacement} {ry_displacement} {rz_displacement} {displacement.total} {self.path}\n")
        self.publisher.publish(displacement)
#        self.get_logger().info(f"x: {acceleration[0]}, y: {acceleration[1]}, z: {acceleration[2]}")
        
            
def main(args=None):
    rclpy.init(args=args)
    subscriber = Data()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
