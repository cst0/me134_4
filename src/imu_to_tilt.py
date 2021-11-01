#!/usr/bin/env python
NO_TF = True # hack for now: hard-code tf

ROLL = 0
PITCH = 1
YAW = 2

import rospy
if not NO_TF:
    import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_about_axis
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Quaternion
from me134.msg import SegwayTilt


class ImuToTilt(object):
    def __init__(self):
        self.imu_sub = rospy.Subscriber("imu_raw", Imu, self.imu_cb, queue_size=5)
        self.tilt_pub = rospy.Publisher("imu_tilt", SegwayTilt, queue_size=5)
        self.tilt_roll_pub = rospy.Publisher("imu/tilt_roll", Float32, queue_size=5)
        self.tilt_pitch_pub = rospy.Publisher("imu/tilt_pitch", Float32, queue_size=5)
        self.tilt_yaw_pub = rospy.Publisher("imu/tilt_yaw", Float32, queue_size=5)

        if not NO_TF:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def imu_cb(self, msg):
        tilt = SegwayTilt()
        tilt.source = 'imu_gyro'
        rpy = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            ])

        self.tilt_roll_pub.publish(Float32(rpy[ROLL]))
        self.tilt_pitch_pub.publish(Float32(rpy[PITCH]))
        self.tilt_yaw_pub.publish(Float32(rpy[YAW]))

        tilt.radians = rpy[PITCH]
        tilt.radians_vel = msg.angular_velocity.y

        self.tilt_pub.publish(tilt)

def main():
    rospy.init_node('imu_to_tilt')
    ImuToTilt()
    rospy.loginfo("imu tilt converter start")
    rospy.spin()
    rospy.loginfo("imu tilt converter shutting down")


if __name__ == "__main__":
    main()
