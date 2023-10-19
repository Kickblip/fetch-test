#!/usr/bin/env python

import copy
import actionlib
import rospy

from math import sin, cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # rospy.loginfo("Waiting for move_base...")
        # self.client.wait_for_server()

    def goto(self, x, y, theta, frame="base_link"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


def rotate_180_degrees():
    move_base.goto(0.0, 0.0, 3.14)


if __name__ == '__main__':
    rospy.init_node("test_arm_poses")

    move_base = MoveBaseClient()

    rospy.sleep(1)

    rotate_180_degrees()

    rospy.spin()
