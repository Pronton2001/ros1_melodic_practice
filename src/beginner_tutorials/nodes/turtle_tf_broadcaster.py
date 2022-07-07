#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename): #callback function?
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     child= turtlename, # name of relative position of turtle
                     parent="world") # name of parent coordinate

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster') 
    turtlename = rospy.get_param('~turtle') # param from launch file
    test = rospy.get_param('~alo') # param from launch file
    print(test)
    rospy.Subscriber('/%s/pose' % turtlename, # get data from, for example, /turtle1/pose and /turtle2/pose. But may be turtle1, turtle2 do not created
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
