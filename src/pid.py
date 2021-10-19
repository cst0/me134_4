#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


def dyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    return config


def pidloop(set_point):
    pass


def main():
    rospy.init_node("test_ddynrec")

    # Create a D(ynamic)DynamicReconfigure
    ddynrec = DDynamicReconfigure("example_dyn_rec")
    # Add variables (name, description, default value, min, max, edit_method)
    ddynrec.add_variable("decimal", "", "Kp", 5.0, -20, 20)
    ddynrec.add_variable("decimal", "", "Ki", 0.0, -20, 20)
    ddynrec.add_variable("decimal", "", "Kd", 0.0, -20, 20)
    ddynrec.add_variable("decimal", "", "upper_limit", 10.0, -20, 20)
    ddynrec.add_variable("decimal", "", "lower_limit", -10.0, -20, 20)
    ddynrec.add_variable("decimal", "", "cutoff_frequency", 20.0, -20, 20)
    ddynrec.add_variable("decimal", "", "target_loop_freqency", 50.0, -20, 20)
    ddynrec.add_variable("decimal", "", "min_loop_frequency", 40.0, -20, 20)

    # Start the server
    ddynrec.start(dyn_rec_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
