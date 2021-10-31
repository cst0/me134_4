#!/usr/bin/env python3

import rospy
from me134.msg import TorsoState, Controller

class JoystickConverter(object):
    def __init__(self):
        rospy.Subscriber('controller', Controller, self.controller_cb)
        self.torso_pub = rospy.Publisher('torso_control', TorsoState, queue_size=5)

    def controller_cb(self, controller_msg:Controller):
        # map controller joysticks to arm movements
        torso_msg = TorsoState()
        torso_msg.chest = controller_msg.axis_state[0]
        torso_msg.left_elbow = controller_msg.axis_state[1]
        torso_msg.head = controller_msg.axis_state[2]
        torso_msg.right_elbow = controller_msg.axis_state[3]

        self.torso_pub.publish(torso_msg)

def main():
    JoystickConverter()
    rospy.spin()


