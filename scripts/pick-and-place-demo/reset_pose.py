#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface


def move_robot():

    move_group = MoveGroupInterface("arm", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    pose = [0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]

    move_group.moveToJointPosition(joints, pose, 0.02)

    result = move_group.get_move_action().get_result()

    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Moved to target pose!")
        else:
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")


if __name__ == '__main__':
    rospy.init_node("reset_arm_poses")

    rospy.sleep(1)

    move_robot()

    rospy.spin()
