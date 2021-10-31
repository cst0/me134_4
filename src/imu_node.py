#!/usr/bin/env python3

import rospy

from math import cos, sin
from sensor_msgs.msg import Imu
from numpy.random import normal as random #type:ignore

class ImuNode(object):
    def __init__(self):
        self.simulation_mode = True
        try:
            import smbus #type:ignore
            from imusensor.MPU9250 import MPU9250 #type:ignore
            self.simulation_mode = False
        except ImportError:
            print("Could not import hardware dependencies. Assuming we're in sim mode and going from here.")

        address = 0x68

        if not self.simulation_mode:
            bus = smbus.SMBus(1) #type:ignore
            self.imu = MPU9250.MPU9250(bus, address) #type:ignore
            self.imu.begin()

        self.imu_pub = rospy.Publisher('imu_raw', Imu, queue_size=3)

    def to_quaternion(self, yaw, pitch, roll):
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = sr * cp * cy - cr * sp * sy;
        q[1] = cr * sp * cy + sr * cp * sy;
        q[2] = cr * cp * sy - sr * sp * cy;
        q[3] = cr * cp * cy + sr * sp * sy;

        return q;

    def loop(self):
        rate = rospy.Rate(30)

        rospy.loginfo("start loop")

        msg = Imu()
        msg.header.frame_id = 'imu'
        while not rospy.is_shutdown():
            try:
                if not self.simulation_mode:
                    self.imu.readSensor()
                    self.imu.computeOrientation()
            except Exception:
                pass

            msg.header.seq += 1
            msg.header.stamp = rospy.Time.now()

            if not self.simulation_mode:
                msg.linear_acceleration.x = self.imu.AccelVals[0]
                msg.linear_acceleration.y = self.imu.AccelVals[1]
                msg.linear_acceleration.z = self.imu.AccelVals[2]

                q = self.to_quaternion(self.imu.yaw, self.imu.pitch, self.imu.roll)

                msg.orientation.x = q[0]
                msg.orientation.y = q[1]
                msg.orientation.z = q[2]
                msg.orientation.w = q[3]

            else:
                msg.linear_acceleration.x = random()
                msg.linear_acceleration.y = random()
                msg.linear_acceleration.z = random()

                msg.orientation.x = random()
                msg.orientation.y = random()
                msg.orientation.z = random()
                msg.orientation.w = random()

            self.imu_pub.publish(msg)

            rate.sleep()

def main():
    rospy.init_node('imu_node', anonymous=False)
    imu = ImuNode()
    imu.loop()

if __name__ == '__main__':
    main()
