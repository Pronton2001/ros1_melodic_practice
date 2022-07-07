import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
import actionlib

from beginner_tutorials.msg import DoDishesAction, DoDishesGoal

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('do_dishes')