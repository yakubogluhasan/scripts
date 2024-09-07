import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction

class LandClient(Node):
    def __init__(self):
        super().__init__('land_client')
        self.client = self.create_client(TelloAction, '/tello_action')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servis bulunamadı, tekrar deniyor...')
        self.req = TelloAction.Request()

    def send_request(self):
        self.req.cmd = 'land'
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    land_client = LandClient()
    land_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(land_client)
        if land_client.future.done():
            response = land_client.future.result()
            land_client.get_logger().info(f'Response: {response}')

            break

    land_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

