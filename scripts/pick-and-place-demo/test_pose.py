#!/usr/bin/env python

import copy
import actionlib
import rospy
from math import sin, cos

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
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


def move_robot():

    move_base.goto(0.0, 0.0, 3.14)

    # move_group = MoveGroupInterface("arm", "base_link")
    # planning_scene = PlanningSceneInterface("base_link")

    # joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
    #           "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    # pose = [-1.57, -0.9, 0.0, 0.9, 0.0, 1.57, 0.0]

    # move_group.moveToJointPosition(joints, pose, 0.02)

    # result = move_group.get_move_action().get_result()

    # if result:
    #     if result.error_code.val == MoveItErrorCodes.SUCCESS:
    #         rospy.loginfo("Moved to target pose!")
    #     else:
    #         rospy.logerr("Arm goal in state: %s",
    #                      move_group.get_move_action().get_state())
    # else:
    #     rospy.logerr("MoveIt! failure no result returned.")


if __name__ == '__main__':
    rospy.init_node("test_arm_poses")

    move_base = MoveBaseClient()

    rospy.sleep(1)

    move_robot()

    rospy.spin()
