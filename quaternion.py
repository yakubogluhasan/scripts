import rclpy


from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('orientation')
        
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.publisher = self.create_publisher(Imu, '/rotation_publisher', 10)
        
    def tf_callback(self, msg):
        for transform in msg.transforms:
            rotation = transform.transform.rotation
            
            quat_list = [rotation.x, rotation.y, rotation.z, rotation.w]
            rot = Rotation.from_quat(quat_list)
            
            [roll, pitch, yaw] = rot.as_euler('xyz', degrees=True)
            
            yaw_error = Float64()
            yaw_error.data = yaw
            self.publisher.publish(yaw_error)
            self.publisher.publish(yaw)
            self.get_logger().info(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
            
def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

