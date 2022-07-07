#!/usr/bin/env python  
import roslib
import rospy
import tf
import csv

if __name__ == '__main__':
    rospy.init_node('turtlebot3_tf_listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    listener.waitForTransform("/map", "/map", rospy.Time(), rospy.Duration(4.0))

    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0)
            listener.waitForTransform("/map", "/base_footprint", now, rospy.Duration(2.0))
            (trans, rot) = listener.lookupTransform("/map", "/base_footprint", now)
            break
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            continue
    print(trans) # pose: x, y; orientation: z, w
    print(rot)
    