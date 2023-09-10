#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Callback function to set target pose
def set_target_pose(msg):
    global target_pose
    target_pose = msg

# Initialize target pose variable
target_pose = None

if __name__ == '__main__':
    rospy.init_node("move_to_target_pose")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Initialize PlanningScene
    planning_scene = PlanningSceneInterface("base_link")

    # Initialize the PoseStamped message
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    # Subscribe to the Pose message topic
    rospy.Subscriber("/target_pose", Pose, set_target_pose)

    while not rospy.is_shutdown():
        if target_pose is not None:
            # Set the current time stamp
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            
            # Set the target pose
            gripper_pose_stamped.pose = target_pose

            # Move the robot arm to the target pose
            move_group.moveToPose(gripper_pose_stamped, "wrist_roll_link")
            result = move_group.get_move_action().get_result()

            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Moved to target pose!")
                else:
                    rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")

            # Reset target pose
            target_pose = None

    # Cancel all movement goals (optional)
    move_group.get_move_action().cancel_all_goals()
