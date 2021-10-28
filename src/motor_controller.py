#!/usr/bin/env python3

import rospy
import joint_state_publisher


def main():
    try:
        rospy.init_node('joint_state_publisher')
        jsp = joint_state_publisher.JointStatePublisher()
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
        jsp.loop()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()

