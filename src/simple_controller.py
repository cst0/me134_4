#!/usr/bin/env python3

import rospy
import numpy as np
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134.msg import SegwayTilt, WheelState
from std_msgs.msg import Float32

class SimpleBalanceController(object):
    def __init__(self, rate=30):
        rospy.init_node('simple_balance', anonymous=False)

        self.ddynrec = DDynamicReconfigure("")
        self.tilt_goal:float = 0.0
        self.kp:float = 1.0
        self.ki:float = 1.0
        self.kd:float = 1.0
        self.sample_timeframe:float = 3.0
        self.ddynrec.add_variable("tilt_goal", "tilt_goal", 0.0, -3.14, 3.14)
        self.ddynrec.add_variable("kp", "kp", 0.0, -2**16, 2**16)
        self.ddynrec.add_variable("ki", "ki", 0.0, -2**16, 2**16)
        self.ddynrec.add_variable("kd", "kd", 0.0, -2**16, 2**16)

        self.ddynrec.add_variable("sample_timeframe", "sample_timeframe", 0.0, 3, 2**16)

        self.servo_min_pwm:int = 0
        self.servo_max_pwm:int = 0
        self.ddynrec.add_variable("servo_min_pwm", "servo_min_pwm", 0, 0, 2**12)
        self.ddynrec.add_variable("servo_max_pwm", "servo_max_pwm", 0, 0, 2**12)

        self.add_variables_to_self()
        self.ddynrec.start(self.dyn_rec_callback)
        self.tilt:SegwayTilt = SegwayTilt()
        rospy.Subscriber('filtered_tilt', SegwayTilt, self.tilt_cb)
        self.error_publisher = rospy.Publisher('controller/error', Float32, queue_size=5)
        self.wheel_goal = rospy.Publisher('wheel_state', WheelState, queue_size=5)

        self.last_err = [0, 0, 0, 0, 0]  # filling with gunk to make sure gradient has something to work with
        self.servo_zero = 2000
        self.servo_left = self.servo_zero
        self.servo_right = self.servo_zero
        self.timer = rospy.Timer(rospy.Duration(1/rate), self.update)

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

    def tilt_cb(self, msg:SegwayTilt):
        self.tilt = msg

    def shutdown(self):
        wheel_msg = WheelState()
        wheel_msg.left_pwm = self.servo_zero
        wheel_msg.right_pwm = self.servo_zero
        self.wheel_goal.publish(wheel_msg)

    def update(self, event):
        del event
        err:float = self.tilt.radians - self.tilt_goal
        err_msg = Float32()
        err_msg.data = err
        self.error_publisher.publish(err_msg)

        # find range of acceptable pwm values around the center, multiply by kp, add, check if valid, execute.
        i = np.trapz(self.last_err)
        d = np.gradient(self.last_err)
        proposed_servo_change = (self.kp * err) + (self.ki * i) + (self.kd * d)
        #rospy.loginfo("<" if proposed_servo_change < 2000 else ">")

        # positive/negative flip to account for motor flip
        self.servo_left = self.servo_zero - proposed_servo_change
        self.servo_right = self.servo_zero + proposed_servo_change

        self.servo_left = max(self.servo_min_pwm, self.servo_left)
        self.servo_left = min(self.servo_max_pwm, self.servo_left)

        self.servo_right = max(self.servo_min_pwm, self.servo_right)
        self.servo_right = min(self.servo_max_pwm, self.servo_right)

        wheel_msg = WheelState()
        # negative to account for motor flip
        wheel_msg.left_pwm =  int(self.servo_left)
        wheel_msg.right_pwm = int(self.servo_right)
        self.wheel_goal.publish(wheel_msg)

        self.last_err.append(err)
        if len(self.last_err) > self.ki_timeframe:
            self.last_err.pop(0)

def main():
    sbc = SimpleBalanceController()

    rospy.loginfo("simple controller ready to go!")
    rospy.on_shutdown(sbc.shutdown)
    rospy.spin()
    rospy.loginfo("simple controller shutting down")

if __name__ == '__main__':
    main()
