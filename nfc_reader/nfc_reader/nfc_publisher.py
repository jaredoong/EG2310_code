import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import busio
import board
import adafruit_amg88xx
import numpy as np

from .pn532 import *
from custom_msgs.msg import Nfc, Button, Thermal

GPIO.setmode(GPIO.BCM)
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set pin 10 to be an input pin and set initial value to be pulled low (off)


class NFCFinder(Node):

    def __init__(self):
        super().__init__('nfc_finder')
        self.publisher_ = self.create_publisher(Nfc, 'nfc_found', 10)
        self.get_logger().info('Created publisher')
        self.get_logger().info('NFC function started')
        # self.nfc_callback()

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
                break
                #self.get_logger().info('NFC found is %s' % msg.nfc_found)
                
        except Exception as e:
            print(e)
        #finally:
        #    GPIO.cleanup()

class ButtonPress(Node):

    def __init__(self):
        super().__init__('button')
        self.publisher_ = self.create_publisher(Button, 'button_pressed', 10)
        self.get_logger().info('Created publisher')
        self.get_logger().info('Button function started')
        # self.button_callback()

    def button_callback(self):
        try:
            
            msg = Button()
            self.button_pressed = False

            while True:
                if GPIO.input(15) != GPIO.LOW:
                    self.get_logger().info("Button not pressed")
                    continue

                self.get_logger().info('Button pressed')
                msg.button_pressed = True
                self.publisher_.publish(msg)
                break
                
        except Exception as e:
            print(e)
        finally:
            GPIO.cleanup()

class ThermalCam(Node):

    def __init__(self):
        super().__init__('thermal_cam')
        self.publisher_ = self.create_publisher(Thermal, 'thermal', 10)
        self.get_logger().info('Created publisher')
        self.get_logger().info('Thermal function started')

        # Seting up the thermal sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c)
        
        # create timer for publisher
        #time_duration = 0.1 # in seconds
        #self.timer = self.create_timer(time_duration, self.thermal_callback)

    def thermal_callback(self):
        try:
            
            msg = Thermal()
            # for breaking out of this function to start launcher
            self.ready = False

            # Seting up the thermal sensor
            #i2c = busio.I2C(board.SCL, board.SDA)
            #amg = adafruit_amg88xx.AMG88XX(i2c)
            
            while True:
                # Sending data to msg
                count = 0
                for row in self.amg.pixels:
                    for col in row:
                        msg.thermal[count] = col
                        count += 1
                self.publisher_.publish(msg)
                self.get_logger().info('Msg published')
                
                
        except Exception as e:
            print(e)
        finally:
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)

    # start with finding NFC
    nfc_pub = NFCFinder()
    nfc_pub.nfc_callback()
    print("Back in main loop")

    # once NFC found, wait for button press
    button_pub = ButtonPress()
    button_pub.button_callback()

    # start the thermal function once button pressed
    thermal_pub = ThermalCam()
    thermal_pub.thermal_callback()

    nfc_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()