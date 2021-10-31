#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_multiply
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from me134.msg import SegwayTilt


class ImuToTilt(object):
    def __init__(self):
        self.imu_sub = rospy.Subscriber("imu_input", Imu, self.imu_cb, queue_size=5)
        self.tilt_pub = rospy.Publisher("imu_tilt", SegwayTilt, queue_size=5)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def imu_cb(self, msg: Imu):
        pose = self.imu_to_pose(msg)
        tilt = SegwayTilt()
        tilt.source = 'imu_gyro'
        tilt.radians = euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
            ], "xyzw")
        tilt.radians_vel = msg.angular_velocity.y

        self.tilt_pub.publish(tilt)

    def imu_to_pose(self, msg: Imu):
        p = PoseStamped()
        p.header = msg.header
        imu_transformation = self.tf_buffer("base_link", "imu", rospy.Time())
        p.pose.orientation = quaternion_multiply(
            imu_transformation.transform.orientation, msg.orientation
        )
        return p


def main():
    itt = ImuToTilt()
    pass


if __name__ == "__main__":
    main()
