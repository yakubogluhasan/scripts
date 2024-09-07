import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Float32 
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tello_msgs.msg import Error

class Landing(Node):
    def __init__(self):
        super().__init__('landing_mission')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.subscription = self.create_subscription(
            Imu,
            '/planar/imu',
            self.PlanarImuCallback,
            qos_profile_sensor_data)
#        self.subscription = self.create_subscription(
#            Imu,
#            '/imu/data',
#            self.DroneImuCallback,
#            qos_profile_sensor_data)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_error_publisher = self.create_publisher(Error, '/pose_error', 10)


        self.kp_yaw = 0.05 # Katsayı değeri (ihtiyaca göre ayarlayabilirsiniz)
        self.Kp = 0.1 # Katsayı değeri (ihtiyaca göre ayarlayabilirsiniz)
        self.kp_altitude = 0.1
        self.kd_yaw = 0.1
        self.ki_yaw = 0.001
        self.ki_x = 0.001
        self.ki = 0.0001
        self.old_yaw = 0.0
        self.yaw_integral_error = 0.0
        self.y_integral_error = 0.0
        self.x_integral_error = 0.0
        self.tag_detected = None
        self.angular_z = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
#        self.drone_odom_pose = Float32()
#        self.planar_odom_pose = Float32()
#        self.distance = Float32()
        self.distance = 0.0
        self.subscription  # prevent unused variable warning



    def tf_callback(self, msg):

        error = Error()
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.tag_detected = False
        for transform in msg.transforms:

            self.tag_detected = True

            rotation = transform.transform.rotation
            translation = transform.transform.translation

            quat_list = [rotation.x, rotation.y, rotation.z, rotation.w]
            rot = Rotation.from_quat(quat_list)
            [roll, pitch, yaw] = rot.as_euler('xyz', degrees=True)

            yaw_error = yaw
            y_error = translation.x
            x_error = translation.y
            z_error = translation.z

            error.x_error = x_error
            error.y_error = y_error
            error.z_error = z_error
            error.yaw_error = yaw_error
            self.pose_error_publisher.publish(error)

            #Kontrol algoritması
            angular_z = cmd_vel_msg.angular.z - self.kp_yaw * yaw_error - self.kd_yaw * (yaw_error - self.old_yaw) - self.ki_yaw * self.yaw_integral_error
            linear_y = cmd_vel_msg.linear.y - self.Kp * y_error - self.ki * self.y_integral_error
            linear_x = cmd_vel_msg.linear.x - self.Kp * x_error - self.ki_x * self.x_integral_error
            linear_z = cmd_vel_msg.linear.z - self.kp_altitude * z_error

            self.old_yaw = yaw_error
            self.yaw_integral_error += yaw_error
            self.y_integral_error += y_error
            self.x_integral_error += x_error

#             Limit control outputs to the range [-1.0, 1.0]
            angular_z = max(min(angular_z, 1.0), -1.0)
            linear_x = max(min(linear_x, 1.0), -1.0)
            linear_y = max(min(linear_y, 1.0), -1.0)  
            linear_z = max(min(linear_z, 1.0), -1.0) 

#            if landing_distance < 3 and landing_distance > 0:
#                cmd_vel_msg.angular.z = self.planar_odom_velocity[3]
#                cmd_vel_msg.linear.y = self.planar_odom_velocity[1]
#                cmd_vel_msg.linear.x = self.planar_odom_velocity[0]
#                cmd_vel_msg.linear.z = -0.1
#            elif landing_distance == 0:
#                cmd_vel_msg.angular.z = 0.0
#                cmd_vel_msg.linear.y = 0.0
#                cmd_vel_msg.linear.x = 0.0
#                cmd_vel_msg.linear.z = 0.0
#            else:
#                #Kontrol mesajı oluşturuluyor
#                cmd_vel_msg.angular.z = angular_z
#                cmd_vel_msg.linear.y = linear_x
#                cmd_vel_msg.linear.x = linear_y
#                cmd_vel_msg.linear.z = linear_z

#            # Limit control outputs to the range [-1.0, 1.0]
#            cmd_vel_msg.angular.z = max(min(cmd_vel_msg.angular.z, 1.0), -1.0)
#            cmd_vel_msg.linear.y = max(min(cmd_vel_msg.linear.y, 1.0), -1.0)
#            cmd_vel_msg.linear.x = max(min(cmd_vel_msg.linear.x, 1.0), -1.0)  
#            cmd_vel_msg.linear.z = max(min(cmd_vel_msg.linear.z, 1.0), -1.0) 


#            Kontrol mesajı oluşturuluyor
            cmd_vel_msg.angular.z = angular_z
            cmd_vel_msg.linear.y = linear_y
            cmd_vel_msg.linear.x = linear_x
            #cmd_vel_msg.linear.z = linear_z

             #Kontrol mesajı yayınlanıyor
            self.cmd_vel_publisher.publish(cmd_vel_msg)

#            self.get_logger().info(f"landing_distance = {landing_distance}")
            self.get_logger().info(f"Control command sent: angular.z = {cmd_vel_msg.angular.z}")
            self.get_logger().info(f"Control command sent: linear.x = {cmd_vel_msg.linear.y}")
            self.get_logger().info(f"Control command sent: linear.y = {cmd_vel_msg.linear.x}")
            self.get_logger().info(f"Control command sent: linear.z = {cmd_vel_msg.linear.z}")


    def PlanarImuCallback(self, msg):
        #self.planar_odom_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.planar_odom_velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z]


#    def DroneImuCallback(self, msg):
#        #self.drone_odom_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
#        self.LandingDistance()
#        self.Controller()
##        
#    def LandingDistance(self):
#        self.distance = math.sqrt((self.drone_odom_pose[0] - self.planar_odom_pose[0])**2 + (self.drone_odom_pose[1] - self.planar_odom_pose[1])**2 + (self.drone_odom_pose[2] - self.planar_odom_pose[2])**2)


#    def Controller(self):

#        cmd_vel = Twist()

#        landing_distance = dist
#        if self.tag_detected == True:
#            if landing_distance < 10.0 and landing_distance > 0.0:
#                cmd_vel.angular.z = self.planar_odom_velocity[3] / 10
#                cmd_vel.linear.y = self.planar_odom_velocity[1] / 10
#                cmd_vel.linear.x = self.planar_odom_velocity[0] / 10
#                cmd_vel.linear.z = -0.1
#            elif landing_distance == 0:
#                cmd_vel.angular.z = 0.0
#                cmd_vel.linear.y = 0.0
#                cmd_vel.linear.x = 0.0
#                cmd_vel.linear.z = 0.0
#            else:
#                cmd_vel.angular.z = self.angular_z
#                cmd_vel.linear.y = self.linear_x
#                cmd_vel.linear.x = self.linear_y
#                cmd_vel.linear.z = self.linear_z
#        else: 
#            self.get_logger().info("Tag not detected!")
#            
#        # Limit control outputs to the range [-1.0, 1.0]
#        cmd_vel.angular.z = max(min(cmd_vel.angular.z, 1.0), -1.0)
#        cmd_vel.linear.y = max(min(cmd_vel.linear.y, 1.0), -1.0)
#        cmd_vel.linear.x = max(min(cmd_vel.linear.x, 1.0), -1.0)  
#        cmd_vel.linear.z = max(min(cmd_vel.linear.z, 1.0), -1.0) 

#         #Kontrol mesajı yayınlanıyor
#        self.cmd_vel_publisher.publish(cmd_vel)

#        self.get_logger().info(f"landing_distance = {landing_distance}")
#        self.get_logger().info(f"Control command sent: angular.z = {cmd_vel.angular.z}")
#        self.get_logger().info(f"Control command sent: linear.x = {cmd_vel.linear.y}")
#        self.get_logger().info(f"Control command sent: linear.y = {cmd_vel.linear.x}")
#        self.get_logger().info(f"Control command sent: linear.z = {cmd_vel.linear.z}")



def main(args=None):
    rclpy.init(args=args)
    landing_node = Landing()
    rclpy.spin(landing_node)
    landing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
