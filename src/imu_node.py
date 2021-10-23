#!/usr/bin/env python3

import rospy

import os
import sys
import time
import smbus
import warnings
from math import sqrt, cos, sin

from imusensor.MPU9250 import MPU9250
from sensor_msgs.msg import Imu
import RPi.GPIO as GPIO

address = 0x68

bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(12, 50)
pwm.start(0)

rospy.init_node('imu_node')
imu_pub = rospy.Publisher('imu_pub', Imu, queue_size=3)

def to_quaternion(yaw, pitch, roll):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q = [0, 0, 0, 0];
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
    q[3] = cr * cp * cy + sr * sp * sy;

    return q;

def main():
    rate = rospy.Rate(30)

    rospy.loginfo("start loop")

    msg = Imu()
    msg.header.frame_id = 'imu'
    while not rospy.is_shutdown():
        try:
            imu.readSensor()
            imu.computeOrientation()
        except Exception as e:
            pass

        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()

        msg.linear_acceleration.x = imu.AccelVals[0]
        msg.linear_acceleration.y = imu.AccelVals[1]
        msg.linear_acceleration.z = imu.AccelVals[2]

        q = to_quaternion(imu.yaw, imu.pitch, imu.roll)

        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        imu_pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
