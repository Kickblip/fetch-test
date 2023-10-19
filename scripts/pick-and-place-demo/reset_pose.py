#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface


class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()


def move_robot():

    head_action = PointHeadClient()

    move_group = MoveGroupInterface("arm", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

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

    head_action.look_at(0.0, 0.0, 0.0, "base_link")


if __name__ == '__main__':
    rospy.init_node("reset_arm_poses")

    rospy.sleep(1)

    move_robot()

    rospy.spin()
