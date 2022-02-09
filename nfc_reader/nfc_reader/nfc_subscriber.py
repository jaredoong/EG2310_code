import rclpy
from rclpy.node import Node

from custom_msgs.msg import Nfc


class NFCSubscriber(Node):

    def __init__(self):
        super().__init__('nfc_subscriber')
        self.subscription = self.create_subscription(
            Nfc,
            'nfc_found',
            self.nfc_callback,
            10)
        self.subscription  # prevent unused variable warning

    def nfc_callback(self, msg):
        self.get_logger().info('NFC detected: "%s"' % msg.nfc_found)
        if msg.nfc_found == True:
            exit()


def main(args=None):
    rclpy.init(args=args)

    nfc_subscriber = NFCSubscriber()

    rclpy.spin(nfc_subscriber)

    nfc_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()