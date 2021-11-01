#!/usr/bin/env python3

import rospy
from me134.msg import SegwayTilt
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from numpy.random import normal as random  # type:ignore

from math import acos, sqrt, cos, sin, asin

class RangeAnglePublisher(object):
    def __init__(self, query_hz=30):
        self.angle_publisher = rospy.Publisher(
            "lidar_tilt", SegwayTilt, queue_size=5
        )
        self.query_hz = query_hz
        self.query_timer = None

        self.sensor_height, self.sensor_xdist, self.sensor_angle = (0,0,0)
        self.ddynrec = DDynamicReconfigure("")
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

        self.SIMULATION_MODE:bool
        try:
            import board #type:ignore
            import busio #type:ignore
            import adafruit_vl53l0x #type:ignore
            i2c = busio.I2C(board.SCL, board.SDA)
            self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)
            self.vl53.measurement_timing_budget = 200000 # set to 200ms reads. slower but more accurate, and it's still faster than we need
            self.SIMULATION_MODE = False
        except ImportError:
            rospy.logwarn("Unable to import hardware content: assuming simulation mode and proceeding.")
            self.SIMULATION_MODE = True

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
        if self.SIMULATION_MODE:
            return float(random())
        else:
            return self.vl53.range * 0.001 # convert from mm to m for ros-standard operation

    def publish_data(self, event):
        del event
        # start by finding the distance between the pivot point and the sensor using user-input data
        sensor_pivot_dist = sqrt(self.sensor_xdist ** 2 + self.sensor_height ** 2)
        # with all three sides, law of sines allows us to find the angle between sensor height and hypotenuse.
        # we'll call that gamma
        a, b, c = self.sensor_xdist, self.sensor_height, sensor_pivot_dist
        gamma = acos((c**2 - a**2 - b**2)/(-2*a*b))
        # we have that top angle. That plus the user-specified angle is the overall top angle of the triangle
        # formed between the pivot, sensor, and measured point.
        # call that theta
        theta = gamma + self.sensor_angle
        # to make use of theta we'll need to know the distance between the pivot and the measured point.
        # a quick law of cosines will do that given our measured distance.
        # We'll call the measured distance d, the hypotenuse from earlier h, and the computed length l.
        h = sensor_pivot_dist
        d = self.get_data()
        L = sqrt(h ** 2 + d ** 2 - 2 * h * d * cos(theta))
        # final step: we have the angle theta and its opposite side, and have
        # the side opposite alpha which is our goal. Law of sines will get us
        # alpha, which is the overall tilt of the platform.
        alpha = asin((d*sin(theta))/L)

        msg = SegwayTilt()
        msg.source = "lidar"
        msg.radians = alpha

        self.angle_publisher.publish(msg)

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
