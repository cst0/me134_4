#!/usr/bin/env python3

import rospy
from me134.msg import WheelState

LEFT_WHEEL = 0
RIGHT_WHEEL = 8

class ServoController(object):
    def __init__(self):
        rospy.init_node('ServoController', anonymous=False)
        rospy.Subscriber('wheel_state', WheelState, self.wheel_state_cb)

        self.simulation_mode = True
        self.i2c_bus = None
        self.pca = None
        try:
            from board import SCL, SDA  # type:ignore
            import busio  # type:ignore
            from adafruit_pca9685 import PCA9685  # type:ignore

            self.i2c_bus = busio.I2C(SCL, SDA)
            self.pca = PCA9685(self.i2c_bus)
            self.pca.frequency = 60
            self.pca.channels[0].duty_cycle = 0x7FFF

            self.simulation_mode = False
        except ImportError:
            print(
                "Could not set up hardware dependencies. Assuming we're in sim mode and going from here."
            )

    def wheel_state_cb(self, msg):
        if self.pca is not None:
            self.pca.channels[LEFT_WHEEL].duty_cycle = msg.left_pwm
            self.pca.channels[RIGHT_WHEEL].duty_cycle = msg.right_pwm
        elif self.simulation_mode:
            rospy.loginfo_once("[MotorController PWM] "+str(msg.left_pwm)+", "+str(msg.right_pwm))
        else:
            rospy.logerr("You tried specifying a motor command, but PCA could not be initialized!")

def main():
    pass


if __name__ == "__main__":
    main()
