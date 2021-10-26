#!/usr/bin/env python3

import rospy
from me134.msg import SegwayTilt
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from numpy.random import normal as random  # type:ignore

SIMULATION_MODE = True


class RangeAnglePublisher(object):
    def __init__(self, query_hz=30):
        self.angle_publisher = rospy.Publisher(
            "distance_angle_measurement", SegwayTilt, queue_size=5
        )
        self.query_hz = query_hz
        self.query_timer = None

        self.ddynrec = DDynamicReconfigure("RangeAngleSensorDynRec")
        self.ddynrec.add_variable(
            "sensor_height", "sensor height from pivot point", 0.05, -20, 20
        )
        self.ddynrec.add_variable(
            "sensor_xdist", "sensor lateral distance from pivot point", 0.05, -20, 20
        )
        self.ddynrec.add_variable(
            "sensor_angle", "sensor angle from ground (rads, 0=down)", 0.05, -20, 20
        )
        self.add_variables_to_self()
        self.ddynrec.start(self.dyn_rec_callback)

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

    def start(self):
        self.query_timer = rospy.Timer(
            rospy.Duration(1 / self.query_hz), self.publish_data
        )

    def get_data(self):
        if SIMULATION_MODE:
            return random()

    def publish_data(self, event):
        del event

    def shutdown(self):
        self.angle_publisher.unregister()
        if self.query_timer is not None:
            self.query_timer.shutdown()


def main():
    rospy.init_node("range_angle_publisher", anonymous=False)
    rap = RangeAnglePublisher()
    rospy.on_shutdown(rap.shutdown)
    rap.start()

    rospy.loginfo("Range angle publisher node running!")
    rospy.spin()
    rospy.loginfo("Range angle publisher node shutting down.")


if __name__ == "__main__":
    main()
