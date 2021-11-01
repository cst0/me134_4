#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134.msg import SegwayTilt, WheelState
from std_msgs.msg import Float32

class SimpleBalanceController(object):
    def __init__(self, rate=30):
        rospy.init_node('simple_balance', anonymous=False)

        self.ddynrec = DDynamicReconfigure("")
        self.tilt_goal:float = 0.0
        self.kp:float = 1.0
        self.ddynrec.add_variable("tilt_goal", "tilt_goal", 0.0, -3.14, 3.14)
        self.ddynrec.add_variable("kp", "kp", 0.0, -2**16, 2**16)

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

        self.servo_position = ((self.servo_max_pwm - self.servo_min_pwm)/2)
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

    def update(self, event):
        del event
        err:float = self.tilt.radians - self.tilt_goal
        err_msg = Float32()
        err_msg.data = err
        self.error_publisher.publish(err_msg)

        # find range of acceptable pwm values around the center, multiply by kp, add, check if valid, execute.
        proposed_servo_change = (self.kp * err) + 2000
        #rospy.loginfo("<" if proposed_servo_change < 2000 else ">")

        self.servo_position = max(self.servo_min_pwm, proposed_servo_change)
        self.servo_position = min(self.servo_max_pwm, self.servo_position)

        wheel_msg = WheelState()
        wheel_msg.left_pwm =  int(self.servo_position)
        wheel_msg.right_pwm = int(self.servo_position)
        self.wheel_goal.publish(wheel_msg)

def main():
    SimpleBalanceController()
    rospy.loginfo("simple controller ready to go!")
    rospy.spin()
    rospy.loginfo("simple controller shutting down")

if __name__ == '__main__':
    main()
