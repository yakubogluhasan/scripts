import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, distance_cm):
        twist_msg = Twist()
        twist_msg.linear.x = 0.3  # İleri hareket etmek için x ekseninde pozitif hız
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Moving forward {distance_cm} cm')
        rclpy.spin_once(self, timeout_sec=distance_cm / 30)  # İleri hareket süresi hesaplama

    def move_backward(self, distance_cm):
        twist_msg = Twist()
        twist_msg.linear.x = -0.3  # Geri hareket etmek için x ekseninde negatif hız
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Moving backward {distance_cm} cm')
        rclpy.spin_once(self, timeout_sec=distance_cm / 30)  # Geri hareket süresi hesaplama

    def move_right(self, distance_cm):
        twist_msg = Twist()
        twist_msg.linear.y = 0.3  # Sağa hareket etmek için y ekseninde negatif hız
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Moving right {distance_cm} cm')
        rclpy.spin_once(self, timeout_sec=distance_cm / 30)  # Sağa hareket süresi hesaplama

    def move_left(self, distance_cm):
        twist_msg = Twist()
        twist_msg.linear.y = -0.3  # Sola hareket etmek için y ekseninde pozitif hız
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Moving left {distance_cm} cm')
        rclpy.spin_once(self, timeout_sec=distance_cm / 30)  # Sola hareket süresi hesaplama

    def move_up(self, distance_cm):
        twist_msg = Twist()
        twist_msg.linear.z = 0.3  # Yukarı hareket etmek için z ekseninde pozitif hız
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Moving up {distance_cm} cm')
        rclpy.spin_once(self, timeout_sec=distance_cm / 30)  # Yukarı hareket süresi hesaplama

    def move_down(self, distance_cm):
        twist_msg = Twist()
        twist_msg.linear.z = -0.3  # Aşağı hareket etmek için z ekseninde negatif hız
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Moving down {distance_cm} cm')
        rclpy.spin_once(self, timeout_sec=distance_cm / 30)  # Aşağı hareket süresi hesaplama

    def stop(self):
        twist_msg = Twist()  # Tüm hız değerlerini sıfırla
        self.publisher_.publish(twist_msg)
        self.get_logger().info('Drone stopped')

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()

    try:
        movements = input("Enter movements (forward_cm backward_cm right_cm left_cm up_cm down_cm): ")
        movements_list = movements.split()

        # İleri gitme
        if len(movements_list) >= 1:
            keyboard_control.move_forward(float(movements_list[0]))

        # Geri gitme
        if len(movements_list) >= 2:
            keyboard_control.move_backward(float(movements_list[1]))

        # Sağa gitme
        if len(movements_list) >= 3:
            keyboard_control.move_right(float(movements_list[2]))

        # Sola gitme
        if len(movements_list) >= 4:
            keyboard_control.move_left(float(movements_list[3]))

        # Yukarı gitme
        if len(movements_list) >= 5:
            keyboard_control.move_up(float(movements_list[4]))

        # Aşağı gitme
        if len(movements_list) >= 6:
            keyboard_control.move_down(float(movements_list[5]))

    except ValueError:
        print("Invalid input! Please enter valid numbers.")

    # Durma komutu
    keyboard_control.stop()

    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

