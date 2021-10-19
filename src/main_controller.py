#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from sensor_msgs.msg import Imu
from me134.msg import SetPoint, PIDOutput


class MainController(object):
    def __init__(self):
        self.current_tilt_state = Imu()
        self.ddynrec = DDynamicReconfigure("MainControllerDynRec")

        # fmt:off
        self.ddynrec.add_variable("Kp",                   "Kp",          5.0,   -20, 20)
        self.ddynrec.add_variable("Ki",                   "Ki",          0.0,   -20, 20)
        self.ddynrec.add_variable("Kd",                   "Kd",          0.0,   -20, 20)
        self.ddynrec.add_variable("upper_com_limit",      "up com lim",  10.0,  -20, 20)
        self.ddynrec.add_variable("lower_com_limit",      "up com lim",  -10.0, -20, 20)
        self.ddynrec.add_variable("target_loop_freqency", "target hz",   50.0,  -20, 20)
        self.ddynrec.add_variable("min_loop_frequency",   "err if less", 40.0,  -20, 20)

        self.ddynrec.add_variable("enable_turning",       "enable turning", True)
        self.ddynrec.add_variable("enable_wheel_balance", "enable balancing via wheel movements", True)
        self.ddynrec.add_variable("enable_tail_balance",  "enable balancing via tail movements", True)

        self.add_variables_to_self()
        self.ddynrec.start(self.dyn_rec_callback)

        self.pid_sub    = rospy.Subscriber("pid_target", SetPoint,  queue_size=1)
        self.tilt_sub   = rospy.Subscriber("tilt_state", Imu,       queue_size=1)
        self.output_pub = rospy.Publisher("pid_output",  PIDOutput, queue_size=1)
        # fmt:on

        rospy.spin()

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

    def tilt_state(self, msg: Imu):
        self.current_tilt_state = msg

    def pidloop(self, msg: SetPoint):
        if self.__dict__["enable_wheel_balance"]:
            pass


if __name__ == "__main__":
    rospy.init_node("MainController")
    mc = MainController()
    rospy.spin()
