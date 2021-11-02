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
        torso_msg.lock_wheels = False
        if any(b for b in controller_msg.button_state[4:8]):
            # one of the triggers is pressed. put us in animatronic mode
            torso_msg.chest = controller_msg.axis_state[0]
            torso_msg.left_elbow = controller_msg.axis_state[1]
            torso_msg.head = controller_msg.axis_state[2]
            torso_msg.right_elbow = controller_msg.axis_state[3]
            torso_msg.lock_wheels = True

        self.torso_pub.publish(torso_msg)

def main():
    rospy.init_node('controller_converter')
    JoystickConverter()
    rospy.spin()


if __name__ == '__main__':
    main()
