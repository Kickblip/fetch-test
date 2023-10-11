#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface


rospy.init_node('torso_head_mover', anonymous=True)


move_group = MoveGroupInterface("torso_and_head", "base_link")
planning_scene = PlanningSceneInterface("base_link")


def move_robot():

    joints = ["torso_lift_joint", "head_tilt_joint"]

    joint_values = [0.3, -0.5]

    move_group.moveToJointPosition(joints, joint_values, wait=True)

    result = move_group.get_move_action().get_result()
    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Robot moved to target joint values!")
        else:
            rospy.logwarn("Failed to move to target joint values!")
    else:
        rospy.logwarn("MoveIt! did not return any result.")

    move_group.get_move_action().cancel_all_goals()


rospy.sleep(1)

move_robot()

rospy.spin()
