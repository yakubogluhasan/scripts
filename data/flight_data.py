import rclpy
from rclpy.node import Node
from tello_msgs.msg import FlightData
from std_msgs.msg import Int32MultiArray
from rclpy.qos import qos_profile_sensor_data

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            FlightData,
            '/flight_data',
            self.data_callback,
            qos_profile_sensor_data)
        self.file = open("velocity_data.txt", "w")  # Metin dosyasını yazmak için aç

    def data_callback(self, msg):
        x_velocity = msg.vgx
        y_velocity = msg.vgy
        z_velocity = msg.vgz
        yaw_degree = msg.yaw
        
        # Verileri dosyaya yaz
        self.file.write(f"{x_velocity} {y_velocity} {z_velocity} {yaw_degree}\n")

    def __del__(self):
        # Dosyayı kapat
        self.file.close()

def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = DataSubscriber()
    rclpy.spin(tf_subscriber)
    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

