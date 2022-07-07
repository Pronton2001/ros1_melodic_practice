#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def turnLeft():
    vel_msg = Twist()
    vel_msg.angular.z = 1.57 / 4 # 1/4pi rad / 2s = 1/2pi (90 degree)
    return vel_msg

def turnRight():
    vel_msg = Twist()
    vel_msg.angular.z = -1.57 / 4 # 1/4pi rad / 2s = 1/2pi (90 degree)
    return vel_msg

def goFoward():
    vel_msg = Twist()
    vel_msg.linear.x =  1 # 0.15 m/s
    return vel_msg

def talker():
    rospy.init_node('robot_controller', anonymous=True) # init publiser named 'talker'????
    # pub = rospy.Publisher('/r2d2_diff_drive_controller/cmd_vel', Twist, queue_size=10) # create publisher to publish to chatter
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # create publisher to publish to chatter
    
    rate = rospy.Rate(2.5) # 2.5hz = 0.4s
    while not rospy.is_shutdown():
        vel_msg = goFoward()
        for _ in range(10): # 10 x 2.5hz = 4s
            pub.publish(vel_msg)
            rate.sleep()
        vel_msg = turnLeft()
        for _ in range(10): # 10 x 2.5hz = 4s
            pub.publish(vel_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
