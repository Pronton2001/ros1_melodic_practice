#! /usr/bin/env python

import rospy
import actionlib
import tf
import csv
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if not os.path.exists('save_points.csv'):
    print("Create sace_points.csv file!!!")
    open('saved_points.csv', 'a')


def get_current_tf():
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
    return trans[:2] + rot[2:] # pose: x, y; orientation: z, w

if __name__ == '__main__':
    rospy.init_node('control_robot')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()

    while 1:
        idx = input("0: save point, 1: Goto Point")
        if idx == 0:
            with open('saved_points.csv','a') as myfile:
                tfWriter = csv.writer(myfile, delimiter=' ')
                name = raw_input('Name:')
                data = get_current_tf()
                tfWriter.writerow([name] + data)
                myfile.flush()
                print('Save successfully point ' + str(name))
        if idx ==2: 
            for id, point in enumerate(saved_points):
                print(str(id) + ':' + str(point[0]))
            name = raw_input("Select by name:")

        if idx == 1:
            saved_points = csv.reader(open('saved_points.csv', 'rb'), delimiter=' ')
            print('List points:')
            for id, point in enumerate(saved_points):
                print(str(id) + ':' + str(point[0]))
            name = raw_input("Select by name:")
            saved_points = csv.reader(open('saved_points.csv', 'rb'), delimiter=' ')
            for point in saved_points:
                if name == point[0]:
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.seq = 11
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = float(point[1])
                    goal.target_pose.pose.position.y = float(point[2])
                    goal.target_pose.pose.orientation.z = float(point[3])
                    goal.target_pose.pose.orientation.w = float(point[4])
                    client.send_goal(goal)
                    client.wait_for_result(rospy.Duration.from_sec(5.0))