#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134.msg import CoMState, SetPoint, PIDOutput


class MainController(object):
    def __init__(self):
        self.current_tilt_state = CoMState()
        self.current_set_point = SetPoint()
        self.ddynrec = DDynamicReconfigure("MainControllerDynRec")
        self.dropped_hz_rate = 0
        self.last_timesteps = []

        # fmt:off
        self.ddynrec.add_variable("Kp",                   "Kp",          5.0,   -20, 20)
        self.ddynrec.add_variable("Ki",                   "Ki",          0.0,   -20, 20)
        self.ddynrec.add_variable("Kd",                   "Kd",          0.0,   -20, 20)
        self.ddynrec.add_variable("upper_com_limit",      "up com lim",  10.0,  -20, 20)
        self.ddynrec.add_variable("lower_com_limit",      "up com lim",  -10.0, -20, 20)
        self.ddynrec.add_variable("target_loop_freqency", "target hz",   50.0,  -20, 20)
        self.ddynrec.add_variable("min_loop_frequency",   "err if less", 40.0,  -20, 20)

        self.ddynrec.add_variable("enable_turning",       "enable turning",                       True)
        self.ddynrec.add_variable("enable_wheel_balance", "enable balancing via wheel movements", True)
        self.ddynrec.add_variable("enable_tail_balance",  "enable balancing via tail movements",  True)

        self.add_variables_to_self()
        self.ddynrec.start(self.dyn_rec_callback)

        self.pid_sub    = rospy.Subscriber("pid_target", SetPoint,  self.pid_target_cb, queue_size=1)
        self.tilt_sub   = rospy.Subscriber("tilt_state", CoMState,  self.tilt_state_cb, queue_size=1)
        self.output_pub = rospy.Publisher("pid_output",  PIDOutput, queue_size=1)
        self.pidloop    = rospy.Timer(rospy.Duration(self.__dict__['target_loop_freqency']), self.pidloop_cb)
        # fmt:on

    def shutdown(self):
        self.pid_sub.unregister()
        self.tilt_sub.unregister()
        self.output_pub.unregister()
        self.pidloop.shutdown()

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

    def tilt_state_cb(self, msg: CoMState):
        self.current_tilt_state = msg

    def pid_target_cb(self, msg: SetPoint):
        self.current_set_point = msg

    def pidloop_cb(self, event):
        if event.last_duration < 1 / self.__dict__["min_loop_frequency"]:
            rospy.logerr_throttle(
                1,
                "PID loop dropped below allowable hz! Last exec time: "
                + str(event.last_duration),
            )

        current_com_pos_error = self.current_tilt_state
        self.store_error(current_com_pos_error)
        kp = self.__dict__["Kp"]
        kd = self.__dict__["Ki"]
        ki = self.__dict__["Kd"]
        target_response = self.get_pid_p(kp) + self.get_pid_i(ki) + self.get_pid_d(kd)

    def store_error(self, current_com_pos_error):
        self.last_timesteps.append(current_com_pos_error)
        if len(self.last_timesteps) > self.__dict__["ki_data_length"]:
            self.last_timesteps.pop()

    def get_pid_p(self, kp) -> float:
        return kp * self.last_timesteps[-1]

    def get_pid_i(self, ki) -> float:
        total_err = 0
        total_ts = 0
        for e in self.last_timesteps:
            total_err += e
            total_ts += 1
        return ki * (total_err / total_ts)

    def get_pid_d(self, kd) -> float:
        # TODO: check me
        return (
            kd * ((self.last_timesteps[-1] + self.last_timesteps[-2]) / 2)
            + self.last_timesteps[-1]
        )


if __name__ == "__main__":
    rospy.init_node("MainController")
    mc = MainController()
    rospy.on_shutdown(mc.shutdown)
    rospy.spin()
