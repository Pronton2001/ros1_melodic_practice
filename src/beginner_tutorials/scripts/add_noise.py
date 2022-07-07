#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import random

pub = None
rate = None

pub = rospy.Publisher('/noisy_odom', Odometry, queue_size=10)

def odomCallback(odom_data):
    odom_data.pose.pose.position.y += random.uniform(-0.5, 0.5)
    pub.publish(odom_data)

if __name__ == '__main__':
    rospy.init_node('add_noise', anonymous=True)
    # rospy.Subscriber('/r2d2_diff_drive_controller/odom', Odometry, odomCallback)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.spin()
