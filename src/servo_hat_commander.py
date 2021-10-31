#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134.msg import WheelState, TorsoState

# constants representing the servo state
# fmt:off
LEFT_WHEEL  = 0
RIGHT_WHEEL = 4
CHEST       = 8
LEFT_ELBOW  = 9
RIGHT_ELBOW = 10
HEAD        = 11
# fmt:on


class ServoController(object):
    def __init__(self):
        rospy.init_node("ServoController", anonymous=False)
        rospy.Subscriber("wheel_state", WheelState, self.wheel_state_cb)
        rospy.Subscriber("torso_control", TorsoState, self.torso_control_cb)

        self.ddynrec = DDynamicReconfigure("")
        self.servo_min_pwm: int = 0
        self.servo_max_pwm: int = 0
        self.ddynrec.add_variable("servo_min_pwm", "servo_min_pwm", 0, 0, 2 ** 16)
        self.ddynrec.add_variable("servo_max_pwm", "servo_max_pwm", 0, 0, 2 ** 16)

        self.add_variables_to_self()
        self.ddynrec.start(self.dyn_rec_callback)

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

    def add_variables_to_self(self):
        var_names = self.ddynrec.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        del level
        var_names = self.ddynrec.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config

    def wheel_state_cb(self, msg: WheelState):
        if self.pca is not None:
            self.pca.channels[LEFT_WHEEL].duty_cycle = msg.left_pwm
            self.pca.channels[RIGHT_WHEEL].duty_cycle = msg.right_pwm
        elif self.simulation_mode:
            rospy.loginfo_once(
                "[MotorController PWM] " + str(msg.left_pwm) + ", " + str(msg.right_pwm)
            )
        else:
            rospy.logerr(
                "You tried specifying a motor command, but PCA could not be initialized!"
            )

    def torso_control_cb(self, msg: TorsoState):
        # fmt:off
        # retrieve values and ensure they're between -1 and 1
        chest       = max(-1, min(1, msg.chest))
        left_elbow  = max(-1, min(1, msg.left_elbow))
        right_elbow = max(-1, min(1, msg.right_elbow))
        head        = max(-1, min(1, msg.head))
        # convert each of these -1 to 1 values into the range of pwm values
        chest       = self.servo_min_pwm + (((self.servo_max_pwm - self.servo_min_pwm)/2) * chest)
        left_elbow  = self.servo_min_pwm + (((self.servo_max_pwm - self.servo_min_pwm)/2) * left_elbow)
        right_elbow = self.servo_min_pwm + (((self.servo_max_pwm - self.servo_min_pwm)/2) * right_elbow)
        head        = self.servo_min_pwm + (((self.servo_max_pwm - self.servo_min_pwm)/2) * head)

        # send pwm values
        if self.pca is not None:
            self.pca.channels[CHEST].duty_cycle       = chest
            self.pca.channels[LEFT_ELBOW].duty_cycle  = left_elbow
            self.pca.channels[RIGHT_ELBOW].duty_cycle = right_elbow
            self.pca.channels[HEAD].duty_cycle        = head
        # fmt:on

def main():
    sc = ServoController()
    rospy.loginfo("spinning servo controller")
    rospy.spin()
    rospy.loginfo("servo controller shutting down")


if __name__ == "__main__":
    main()
