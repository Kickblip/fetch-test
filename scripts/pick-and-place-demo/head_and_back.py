#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('spine_head_commander', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "head_torso_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    joint_goal = move_group.get_current_joint_values()
    joint_names = move_group.get_joints()

    if "torso_lift_joint" in joint_names:
        index = joint_names.index("torso_lift_joint")
        joint_goal[index] = 0.2

    if "head_tilt_joint" in joint_names:
        index = joint_names.index("head_tilt_joint")
        joint_goal[index] = -0.5

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
