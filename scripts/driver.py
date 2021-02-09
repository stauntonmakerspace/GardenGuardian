#!/usr/bin/env python3.6
# https://maker.pro/nvidia-jetson/tutorial/how-to-use-gpio-pins-on-jetson-nano-developer-kit
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import board
import busio
import time
import sys
import rospy
from geometry_msgs.msg import Twist
# Set the GPIO modes
GPIO.setwarnings(False)

# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...

print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL, board.SDA))
print("Initializing ServoKit")
kit = ServoKit(channels=16, i2c=i2c_bus0)
print("Done initializing")

def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


class Motor:
    def __init__(self, servo_num, direction_pinA, direction_pinB):
        self._servo_num = servo_num
        self._direction_pinA = direction_pinA
        self._direction_pinB = direction_pinB

        GPIO.setup(direction_pinA, GPIO.OUT)
        GPIO.setup(direction_pinB, GPIO.OUT)

      
    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 180)
        kit.servo[self._servo_num].angle = speed
        # Positive speeds move wheels forward, negative speeds move wheels backward
        GPIO.output(self._direction_pinA, GPIO.HIGH if speed_percent > 0 else GPIO.LOW)
        GPIO.output(self._direction_pinB, GPIO.LOW if speed_percent > 0 else GPIO.HIGH)


class Driver:
    def __init__(self):
        rospy.init_node('base_controller')

        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 3)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 1)
        self._wheel_base = rospy.get_param('~wheel_base', 0.091)

        # Assign pins to motors. These may be distributed
        # differently depending on how you've built your robot
        self._left_motor = Motor(0, 'LCD_TE', 'SPI2_SCK')
        self._right_motor = Motor(1, 'SPI1_MISO', 'SPI1_MOSI')
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for velocity twist message
        rospy.Subscriber(
            'cmd_vel', Twist, self.velocity_received_callback)

    def velocity_received_callback(self, message):
        """Handle new velocity command message."""

        self._last_received = rospy.get_time()

        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (
            180 * left_speed/self._max_speed)
        self._right_speed_percent = (
            180 * right_speed/self._max_speed)

    def run(self):
        """The control loop of the driver."""

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop
            # moving
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                self._left_motor.move(self._left_speed_percent)
                self._right_motor.move(self._right_speed_percent)
            else:
                self._left_motor.move(0)
                self._right_motor.move(0)
            rate.sleep()

if __name__ == '__main__':
    try:
        driver = Driver()
        # Run driver. This will block
        driver.run()
    finally:
        pass # GPIO.cleanup()

