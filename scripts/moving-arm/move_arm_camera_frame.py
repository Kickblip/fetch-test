#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from fetch_test.srv import MoveToPose, MoveToPoseResponse
import tf


def handle_move_to_pose(req):

    # Create a response object
    res = MoveToPoseResponse()

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "head_camera_link")

    # Initialize PlanningScene
    planning_scene = PlanningSceneInterface("head_camera_link")

    # Initialize TF listener
    listener = tf.TransformListener()

    # Wait for TF to initialize
    rospy.sleep(2)

    # Ensure that the request pose is in 'base_link'
    if req.target_pose.header.frame_id != 'base_link':
        rospy.logerr("Received pose is not in 'base_link'")
        res.success = False
        return res

    try:
        # Transform the pose from base_link to head_camera_link
        pose_in_camera_link = listener.transformPose(
            'head_camera_link', req.target_pose)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr(
            "Failed to transform pose from base_link to head_camera_link")
        res.success = False
        return res

    # Move the robot arm to the target pose
    move_group.moveToPose(pose_in_camera_link, "wrist_roll_link")
    result = move_group.get_move_action().get_result()

    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Moved to target pose!")
            res.success = True
        else:
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
            res.success = False
    else:
        rospy.logerr("MoveIt! failure no result returned.")
        res.success = False

    return res


if __name__ == '__main__':
    rospy.init_node("move_to_target_pose_service")

    # Initialize the service
    s = rospy.Service('move_to_pose', MoveToPose, handle_move_to_pose)

    rospy.spin()
