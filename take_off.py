import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction

class TakeoffClient(Node):
    def __init__(self):
        super().__init__('takeoff_client')
        self.client = self.create_client(TelloAction, '/tello_action')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servis bulunamadı, tekrar deniyor...')
        self.req = TelloAction.Request()

    def send_request(self):
        self.req.cmd = 'takeoff'
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    takeoff_client = TakeoffClient()
    takeoff_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(takeoff_client)
        if takeoff_client.future.done():
            response = takeoff_client.future.result()
            takeoff_client.get_logger().info(f'Response: {response}')

            break

    takeoff_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

