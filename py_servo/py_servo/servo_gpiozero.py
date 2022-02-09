import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

# to prevent servo jitter
factory = PiGPIOFactory()

# Sets the initial angle to 15, and allows servo to turn from 0 to 180
s = AngularServo(18, initial_angle=15, min_angle=0, max_angle=180, min_pulse_width=0.0006, max_pulse_width=0.0024, pin_factory=factory)

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
        # find distance of object in front of front 10 angles
        front_distance = laser_range[:4] + laser_range[-4:]
        # take the average distance
        avg_dis = np.mean(front_distance)

        # log the avgerage distance
        self.get_logger().info('Average distance in front is %f m, Angle of servo is %f' % (avg_dis, s.angle))
        # once the average distance is approx 1m, activate the servo
        if avg_dis <= 1.00:
            # Set servo angle to 165
            s.angle = 165



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