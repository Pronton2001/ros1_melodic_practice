#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# footprint = [[-0.105, -0.105], [-0.105, 0.105], [0.041, 0.105], [0.041, -0.105]]
footprint = [[-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]
right_bottom_corner = footprint[0] # [x,y]
left_bottom_corner = footprint[1]
left_top_corner = footprint[2]
right_top_corner = footprint[3]
DANGER_RANGE = 1

def goForward(vel_msg):
    vel_msg.linear.x = 0.5
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0

def RotateLeft(vel_msg):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0.4

def RotateRight(vel_msg):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.angular.z = -0.4

def goBackward(vel_msg):
    vel_msg.linear.x = -1
    vel_msg.angular.z = -1

def stop(vel_msg):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0


def callback(data):
    # rospy.init_node('robot_controller', anonymous=True) # init publiser named 'talker'????
    front, left, right, back = False, False, False, False
    pub = rospy.Publisher('/r2d2_diff_drive_controller/cmd_vel', Twist, queue_size=10) # create publisher to publish to chatter
    for i in range(360):
        r = data.ranges[i]
        if r < DANGER_RANGE:
            radius_angle = i / 180.0 * math.pi
            # x go up(head), y go left, radius 0 at positive x, counter-clockwise
            x = r * math.cos(radius_angle)
            y = r * math.sin(radius_angle)
            if obsFront(x, y):
                front = True
            elif obsLeft(x, y):
                left = True
            elif obsRight(x,y):
                right = True
            elif obsBack(x,y):
                back = True
    vel_msg = Twist()
    if not front:
        goForward(vel_msg)
    else: 
        stop(vel_msg)
        if left and right:
            pass
        elif right:
            RotateLeft(vel_msg)
        else: RotateRight(vel_msg)

    pub.publish(vel_msg)
        # else: print('NO')

def obsBack(x, y):
    # if x <= 0 and x >= -DANGER_RANGE and y <= 0.105 and y >= -0.105:
    return x <= 0 and x >= -DANGER_RANGE and y <= left_bottom_corner[1] and y >= right_bottom_corner[1]

def obsFront(x, y):
    return x >= 0 and x <= DANGER_RANGE and y <= left_bottom_corner[1] and y >= right_bottom_corner[1]

def obsRight(x, y):
    return x <= right_top_corner[0] and x >= right_bottom_corner[0] and y >= -DANGER_RANGE and y <= 0
     
def obsLeft(x, y):
    return x <= right_top_corner[0] and x >= right_bottom_corner[0]  and y >= 0 and y <= DANGER_RANGE

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('feedback', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
