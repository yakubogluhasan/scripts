import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Float32MultiArray
#from rclpy.qos import qos_profile_sensor_data

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
#        self.subscription = self.create_subscription(
#            Odometry,
#            '/planar/odom',
#            self.odom_callback,
#            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tag_detected = None
        self.kp = 0.01 # Katsayı değeri (ihtiyaca göre ayarlayabilirsiniz)
        self.KP = 0.2# Katsayı değeri (ihtiyaca göre ayarlayabilirsiniz)
        self.kp_z = 0.1
        self.yaw_error = 0
        self.ki_yaw = 0.01
        self.ki = 0.1
        self.kd = 0.001
        self.x_error = 0.0
        self.y_error = 0.0
        self.subscription  # prevent unused variable warning
  
#    def odom_callback(self, msg):
#        self.planar_odom = Float32MultiArray()
#        self.planar_odom = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
#        #Planar_Odom = msg.twist
#        #self.planar_odom = Float32MultiArray()
#        #self.planar_odom = [Planar_Odom.twist.linear.x, Planar_Odom.twist.linear.y, Planar_Odom.twist.angular.z]
#        
    def tf_callback(self, msg):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        yaw_integral_error = 0
        integral_error_x = 0.0
        integral_error_y = 0.0
        old_x_error = 0.0
        old_y_error = 0.0
        timer = 0
        self.tag_detected = False
        for transform in msg.transforms:
            self.tag_detected = True
            rotation = transform.transform.rotation
            translation = transform.transform.translation
            
            quat_list = [rotation.x, rotation.y, rotation.z, rotation.w]
            rot = Rotation.from_quat(quat_list)
            [roll, pitch, yaw] = rot.as_euler('xyz', degrees=True)
            
            
#            self.get_logger().info(f"old yaw = {old_}, yaw error = {self.yaw_error}")
            self.yaw_error = yaw  # yaw değeri alınıyor
            self.x_error = translation.x
            self.y_error = translation.y
            
            old_x_error = self.x_error
            old_y_error = self.y_error
#            z_error = translation.z
            if timer >= 100 :
                z_error = translation.z
            else:
                z_error = 0
            
            yaw_integral_error += self.yaw_error        
            integral_error_x += self.x_error
            integral_error_y += self.y_error
            #Kontrol algoritması
            angular_z = cmd_vel_msg.angular.z - self.kp * self.yaw_error - self.ki_yaw * (yaw_integral_error)
            linear_x = cmd_vel_msg.linear.y - self.KP * self.x_error - self.ki * (integral_error_x) - self.kd * (self.x_error - old_x_error)
            linear_y = cmd_vel_msg.linear.x - self.KP * self.y_error - self.ki * (integral_error_y) - self.kd * (self.y_error - old_y_error)
            linear_z = cmd_vel_msg.linear.z - self.kp_z * z_error
            
            # Limit control outputs to the range [-1.0, 1.0]
            angular_z = max(min(angular_z, 1.0), -1.0)
            linear_x = max(min(linear_x, 1.0), -1.0)
            linear_y = max(min(linear_y, 1.0), -1.0)
            linear_z = max(min(linear_z, 1.0), -1.0)   
            
            
            #linear_x = self.planar_odom[0]
            #linear_y = self.planar_odom[1]
            #angular_z = self.planar_odom[2]
            
            
             #Kontrol mesajı oluşturuluyor
            cmd_vel_msg.angular.z = angular_z
            cmd_vel_msg.linear.y = linear_x
            cmd_vel_msg.linear.x = linear_y
            cmd_vel_msg.linear.z = linear_z
            
            
        
             #Kontrol mesajı yayınlanıyor
            self.publisher.publish(cmd_vel_msg)
            self.get_logger().info(f"Control command sent: angular.z = {cmd_vel_msg.angular.z}")
            self.get_logger().info(f"Control command sent: linear.x = {cmd_vel_msg.linear.x}")
            self.get_logger().info(f"Control command sent: linear.y = {cmd_vel_msg.linear.y}")
            self.get_logger().info(f"Control command sent: linear.z = {cmd_vel_msg.linear.z}")
            timer = timer + 1
            
def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

