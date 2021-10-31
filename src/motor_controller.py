#!/usr/bin/env python3

import rospy
import joint_state_publisher
import sys
from sensor_msgs.msg import JointState

jsp = joint_state_publisher.JointStatePublisher()
def main():
    try:
        rospy.init_node('joint_state_publisher')
        rospy.loginfo(jsp.joint_list)
        for name in jsp.free_joints.keys():
            rospy.loginfo(str(name)+" "+str(jsp.free_joints[name]))
            jsp.free_joints[name]['position'] += 1
        #for joint_str in jsp.joint_list:
        #rospy.loginfo(jsp.joint_info['tilt_joint'])
#        if use_gui:
#            rospy.logwarn("The 'use_gui' parameter was specified, which is deprecated.  We'll attempt to find and run the GUI, but if this fails you should install the 'joint_state_publisher_gui' package instead and run that.  This backwards compatibility option will be removed in Noetic.")
#            try:
#                import signal
#                import threading
#
#                import joint_state_publisher_gui
#
#                from python_qt_binding.QtWidgets import QApplication
#
#                app = QApplication(sys.argv)
#
#                num_rows = joint_state_publisher.get_param('num_rows', 0)
#                jsp_gui = joint_state_publisher_gui.JointStatePublisherGui('Joint State Publisher',
#                                                                           jsp, num_rows)
#                jsp_gui.show()
#                jsp_gui.sliderUpdateTrigger.emit()
#
#                threading.Thread(target=jsp_gui.jsp.loop).start()
#                signal.signal(signal.SIGINT, signal.SIG_DFL)
#                sys.exit(app.exec_())
#            except ImportError:
#                rospy.logerr("Could not find the GUI, install the 'joint_state_publisher_gui' package")
#                sys.exit(1)
#        else:
        loop_once()

    except rospy.ROSInterruptException:
        pass

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

def loop_once():
    hz = get_param("rate", 10)  # 10hz
    r = rospy.Rate(hz)

    delta = get_param("delta", 0.0)

    # Publish Joint States
    msg = JointState()
    msg.header.stamp = rospy.Time.now()

    if delta > 0:
        jsp.update(delta)

    # Initialize msg.position, msg.velocity, and msg.effort.
    has_position = len(jsp.dependent_joints.items()) > 0
    has_velocity = False
    has_effort = False
    for name, joint in jsp.free_joints.items():
        if not has_position and 'position' in joint:
            has_position = True
        if not has_velocity and 'velocity' in joint:
            has_velocity = True
        if not has_effort and 'effort' in joint:
            has_effort = True
    num_joints = (len(jsp.free_joints.items()) +
                  len(jsp.dependent_joints.items()))
    if has_position:
        msg.position = num_joints * [0.0]
    if has_velocity:
        msg.velocity = num_joints * [0.0]
    if has_effort:
        msg.effort = num_joints * [0.0]

    for i, name in enumerate(jsp.joint_list):
        msg.name.append(str(name))
        joint = None

        # Add Free Joint
        if name in jsp.free_joints:
            joint = jsp.free_joints[name]
            factor = 1
            offset = 0
        # Add Dependent Joint
        elif name in jsp.dependent_joints:
            param = jsp.dependent_joints[name]
            parent = param['parent']
            factor = param.get('factor', 1)
            offset = param.get('offset', 0)
            # Handle recursive mimic chain
            recursive_mimic_chain_joints = [name]
            while parent in jsp.dependent_joints:
                if parent in recursive_mimic_chain_joints:
                    error_message = "Found an infinite recursive mimic chain"
                    rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                    sys.exit(1)
                recursive_mimic_chain_joints.append(parent)
                param = jsp.dependent_joints[parent]
                parent = param['parent']
                offset += factor * param.get('offset', 0)
                factor *= param.get('factor', 1)
            joint = jsp.free_joints[parent]

        if has_position and 'position' in joint:
            msg.position[i] = joint['position'] * factor + offset
        if has_velocity and 'velocity' in joint:
            msg.velocity[i] = joint['velocity'] * factor
        if has_effort and 'effort' in joint:
            msg.effort[i] = joint['effort']

    if msg.name or msg.position or msg.velocity or msg.effort:
        # Only publish non-empty messages
        jsp.pub.publish(msg)
    try:
        r.sleep()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        pass


if __name__ == '__main__':
    main()

