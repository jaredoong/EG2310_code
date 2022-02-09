import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import RPi.GPIO as GPIO
import pigpio

# Setting the servo pin based on BCM
servo = 18

# pigpio used to prevent servo jitter
p = pigpio.pi()
p.set_mode(servo, pigpio.OUTPUT)
p.set_PWM_frequency(servo, 50)

# Setting the initial angle at 15
p.set_servo_pulsewidth(servo, 666.67)

class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # calculate distance of +- 5 
        front_distance = laser_range[:4] + laser_range[-4:]
        # take the average distance
        avg_dis = np.mean(front_distance)

        # log the avgerage distance
        self.get_logger().info('Average distance in front is %f m' % avg_dis)

        # once the average distance is approx 1m, activate the servo
        if avg_dis <= 1.00:
            # Setting the angle to 165
            p.set_servo_pulsewidth(servo, 2333.33)



def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()

    rclpy.spin(scanner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()