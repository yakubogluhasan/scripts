import rclpy
import subprocess
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation


class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
                
        self.kp   = 0.25
        self.kP   = 0.007
        
        self.ki   = 0.005
        self.kI   = 0.0001
        
        self.kd   = 15
        self.kD   = 0.0001
        
        self.kp_z = 0.1

        self.yaw_error = 0
        self.x_error = 0.0
        self.y_error = 0.0
        self.subscription  # prevent unused variable warning
        
        #self.timer = 0
        
    def tf_callback(self, msg):
        cmd_vel_msg = Twist()
        
        old_yaw_error = 0.0
        old_x_error   = 0.0
        old_y_error   = 0.0
        
        integral_error_yaw = 0.0
        integral_error_x   = 0.0
        integral_error_y   = 0.0
        
        self.tag_detected = False
                
        for transform in msg.transforms:
            self.tag_detected = True
            if transform.child_frame_id == 'tag36h11_0' and transform.transform.translation.z > 0.7:
                rotation    = transform.transform.rotation
                translation = transform.transform.translation
                
                quat_list = [rotation.x, rotation.y, rotation.z, rotation.w]
                rot = Rotation.from_quat(quat_list)
                [roll, pitch, yaw] = rot.as_euler('xyz', degrees=True)
                
                self.yaw_error = yaw
                self.x_error   = translation.x
                self.y_error   = translation.y
                z_error = translation.z
                
                old_yaw_error = self.yaw_error
                old_x_error   = self.x_error
                old_y_error   = self.y_error
                
#                if self.timer >= 500:
#                    z_error = translation.z
#                else:
#                    z_error = 0
                
                integral_error_yaw += self.yaw_error        
                integral_error_x   += self.x_error
                integral_error_y   += self.y_error
                
                #Kontrol algoritması
                angular_z = cmd_vel_msg.angular.z - self.kP   * self.yaw_error - self.kI * (integral_error_yaw) - self.kD * (self.yaw_error - old_yaw_error)
                linear_x  = cmd_vel_msg.linear.y  - self.kp   * self.x_error   - self.ki * (integral_error_x)   - self.kd * (self.x_error   - old_x_error)
                linear_y  = cmd_vel_msg.linear.x  - self.kp   * self.y_error   - self.ki * (integral_error_y)   - self.kd * (self.y_error   - old_y_error)
                linear_z  = cmd_vel_msg.linear.z  - self.kp_z * z_error
                
                #Limit control outputs to the range [-1.0, 1.0]
                angular_z = max(min(angular_z, 1.0), -1.0)
                linear_x  = max(min(linear_x, 1.0), -1.0)
                linear_y  = max(min(linear_y, 1.0), -1.0)
                linear_z  = max(min(linear_z, 1.0), -1.0)   
    
                #Kontrol mesajı oluşturuluyor
                cmd_vel_msg.angular.z = angular_z
                cmd_vel_msg.linear.y  = linear_x
                cmd_vel_msg.linear.x  = linear_y
                cmd_vel_msg.linear.z  = linear_z
                
                #Kontrol mesajı yayınlanıyor
                self.publisher.publish(cmd_vel_msg)
                self.get_logger().info("----------------------------------------------------------")
                self.get_logger().info(f"Control command sent: angular.z = {cmd_vel_msg.angular.z}")
                self.get_logger().info(f"Control command sent: linear.x  = {cmd_vel_msg.linear.x}")
                self.get_logger().info(f"Control command sent: linear.y  = {cmd_vel_msg.linear.y}")
                self.get_logger().info(f"Control command sent: linear.z  = {cmd_vel_msg.linear.z}")
                self.get_logger().info("----------------------------------------------------------")
                #self.timer = self.timer + 1
            
            elif transform.child_frame_id == 'tag36h11_1' and transform.transform.translation.z < 0.7:
                rotation    = transform.transform.rotation
                translation = transform.transform.translation
                
                quat_list = [rotation.x, rotation.y, rotation.z, rotation.w]
                rot = Rotation.from_quat(quat_list)
                [roll, pitch, yaw] = rot.as_euler('xyz', degrees=True)
                
                self.yaw_error = yaw
                self.x_error   = translation.x
                self.y_error   = translation.y
                z_error = translation.z
                
                old_yaw_error = self.yaw_error
                old_x_error   = self.x_error
                old_y_error   = self.y_error
                                
                integral_error_yaw += self.yaw_error        
                integral_error_x   += self.x_error
                integral_error_y   += self.y_error
                
                #Kontrol algoritması
                angular_z = cmd_vel_msg.angular.z - self.kP   * self.yaw_error - self.kI * (integral_error_yaw) - self.kD * (self.yaw_error - old_yaw_error)
                linear_x  = cmd_vel_msg.linear.y  - self.kp   * self.x_error   - self.ki * (integral_error_x)   - self.kd * (self.x_error   - old_x_error)
                linear_y  = cmd_vel_msg.linear.x  - self.kp   * self.y_error   - self.ki * (integral_error_y)   - self.kd * (self.y_error   - old_y_error)
                linear_z  = cmd_vel_msg.linear.z  - self.kp_z * z_error
                
                #Limit control outputs to the range [-1.0, 1.0]
                angular_z = max(min(angular_z, 1.0), -1.0)
                linear_x  = max(min(linear_x, 1.0), -1.0)
                linear_y  = max(min(linear_y, 1.0), -1.0)
                linear_z  = max(min(linear_z, 1.0), -1.0)
    
                #Kontrol mesajı oluşturuluyor
                cmd_vel_msg.angular.z = angular_z
                cmd_vel_msg.linear.y  = linear_x
                cmd_vel_msg.linear.x  = linear_y
                cmd_vel_msg.linear.z  = linear_z
                
                #Kontrol mesajı yayınlanıyor
                self.publisher.publish(cmd_vel_msg)
                self.get_logger().info("-----------------------------Landing-----------------------------")
                self.get_logger().info(f"Control command sent: angular.z = {cmd_vel_msg.angular.z}")
                self.get_logger().info(f"Control command sent: linear.x  = {cmd_vel_msg.linear.x}")
                self.get_logger().info(f"Control command sent: linear.y  = {cmd_vel_msg.linear.y}")
                self.get_logger().info(f"Control command sent: linear.z  = {cmd_vel_msg.linear.z}")
                self.get_logger().info("-----------------------------Landing-----------------------------")
                
#                if(transform.transform.translation.z < 0.5):
#                    subprocess.run(["python3", "land.py"])
#                    self.get_logger().info("Landing initiated. Shutting down node.")
#                    subprocess.run(command, shell=True)

        if(self.tag_detected == False):
            #Kontrol mesajı yayınlanıyor
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_msg.linear.y  = 0.0
            cmd_vel_msg.linear.x  = 0.0
            cmd_vel_msg.linear.z  = 0.0
            self.publisher.publish(cmd_vel_msg)
            self.get_logger().info("TAG NOT DETECTED !!! SEND 0.0 FOR ALL")
                

def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
