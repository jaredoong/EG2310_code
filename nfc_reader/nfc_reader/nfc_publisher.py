"""
This example shows connecting to the PN532 with I2C (requires clock
stretching support), SPI, or UART. SPI is best, it uses the most pins but
is the most reliable and universally supported.
After initialization, try waving various 13.56MHz RFID cards over it!
"""
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Nfc

import RPi.GPIO as GPIO

from .pn532 import *

class NFCFinder(Node):

    def __init__(self):
        super().__init__('nfc_finder')
        self.publisher_ = self.create_publisher(Nfc, 'nfc_found', 10)
        self.get_logger().info('Created publisher')
        self.get_logger().info('NFC function started')
        self.nfc_callback()

    def nfc_callback(self):
        try:
            #pn532 = PN532_SPI(debug=False, reset=20, cs=4)
            pn532 = PN532_I2C(debug=False, reset=20, req=16)
            #pn532 = PN532_UART(debug=False, reset=20)

            #ic, ver, rev, support = pn532.get_firmware_version()
            #print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))

            # Configure PN532 to communicate with MiFare cards
            pn532.SAM_configuration()
            
            msg = Nfc()
            self.nfc_found = False

            #self.get_logger().info('Beginning while loop')
            while True:
                # Check if a card is available to read
                uid = pn532.read_passive_target(timeout=0.5)
                
                self.publisher_.publish(msg)
                # Try again if no card is available.
                if uid is None:
                    self.get_logger().info('NFC not found')
                    continue
                self.get_logger().info('NFC found')
                msg.nfc_found = True
                self.publisher_.publish(msg)
                exit()
                #self.get_logger().info('NFC found is %s' % msg.nfc_found)
                
        except Exception as e:
            print(e)
        finally:
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)

    nfc_pub = NFCFinder()

    rclpy.spin(nfc_pub)

    nfc_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()