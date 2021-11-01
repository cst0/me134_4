#!/usr/bin/env python3

import rospy
import numpy as np
import time
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from filterpy.kalman import KalmanFilter as kf #type:ignore
from std_msgs.msg import Empty
from me134.msg import SegwayTilt


class KalmanFilter(object):
    def __init__(self, rate=30):
        self.ddynrec = DDynamicReconfigure("")
        self.kalman = None

        self.segway_tilt_input_topics = ""
        self.ddynrec.add_variable(
            "segway_tilt_input_topics", "comma separated list of segway tilt inputs", ""
        )

        self.add_variables_to_self()
        self.ddynrec.start(self.dyn_rec_callback)

        self.update_inputs_trigger = rospy.Subscriber(
            "kalman_update_input_sources", Empty, self.update_inputs
        )
        self.filtered_pub = rospy.Publisher('filtered_tilt', SegwayTilt, queue_size=5)
        self.tilt_subscribers = {}
        self.tilt_data = {}
        self.update_inputs(Empty())

        rospy.Timer(rospy.Duration(1/rate), self.timed_cb)

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

    def update_inputs(self, msg):
        del msg
        rospy.loginfo("told to update input sources ["+self.segway_tilt_input_topics+"]")
        # for all possible input sources, strip whitespace and split on commas
        tilt_topics = self.segway_tilt_input_topics.replace(" ", "").split(",")
        valid_tilt_topics = self.query_topics(tilt_topics, SegwayTilt)

        self.kalman = kf(dim_x=2, dim_z=len(valid_tilt_topics))
        self.kalman.x = np.array([0,0])
        self.kalman.F = np.array([[1., 1.],[0.,1.]])
        self.kalman.H = np.array([[1., 0.]])
        self.kalman.P *= 1000
        self.kalman.R = 5

        self.delete_input_subscribers()
        self.create_input_subscribers(valid_tilt_topics)

    def query_topics(self, topic_list, msg_type):
        valid_topics = []
        for topic in topic_list:
            try:
                data = rospy.wait_for_message(topic, msg_type, timeout=2)
                if data is not None:
                    valid_topics.append(topic)
            except Exception as e:
                rospy.logerr(e)

        while len(valid_topics) != len(topic_list):
            rospy.logerr("Not all topics are sending data (yet). Trying again.")
            valid_topics = self.query_topics(topic_list, msg_type)

        rospy.loginfo("Streaming data from "+str(valid_topics))
        return valid_topics

    def delete_input_subscribers(self):
        for k in self.tilt_subscribers.keys():
            self.tilt_subscribers[k].unregister()

    def create_input_subscribers(self, tilt_topics):
        for t in tilt_topics:
            rospy.loginfo('Creating subscriber on topic '+t)
            self.tilt_subscribers[t] = rospy.Subscriber(t, SegwayTilt, self.tilt_cb)

    def tilt_cb(self, msg:SegwayTilt):
        self.tilt_data[msg.source] = msg.radians

    def timed_cb(self, event):
        del event
        measurements = []
        for k in self.tilt_data.keys():
            m = self.tilt_data.get(k)
            if m is not None:
                measurements.append(m)

        if self.kalman is not None:
            self.kalman.predict()
            #rospy.loginfo(measurements)
            try:
                self.kalman.update(np.array(measurements).reshape((len(measurements), 1)))
            except ValueError as e:
                rospy.logwarn("tried calling filtering before data received")
                return  # no measurements yet
            filtered = SegwayTilt()
            filtered.source = 'filter'
            filtered.radians = self.kalman.x[0]
            filtered.radians_vel = self.kalman.x[1]
            self.filtered_pub.publish(filtered)

    def shutdown(self):
        self.delete_input_subscribers()
        self.filtered_pub.unregister()

def main():
    rospy.init_node("tilt_kalman_filter", anonymous=False)
    kf = KalmanFilter()
    rospy.on_shutdown(kf.shutdown)
    rospy.loginfo("Kalman filtering ready to go!")
    rospy.spin()
    rospy.loginfo("Kalman filter shutdown")


if __name__ == "__main__":
    main()
