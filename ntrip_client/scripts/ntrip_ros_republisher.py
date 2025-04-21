#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import random

pub = rospy.Publisher("robot/ublox/fix", NavSatFix, queue_size=1)

def republishCb(msg):
    pub.publish(msg)

def listener():
    rospy.init_node('ublox_fix_republisher')
    rospy.Subscriber("ublox/fix", NavSatFix, republishCb)
    rospy.spin()

if __name__ == '__main__':
    print("Este republish arregla todo del gps")
    listener()