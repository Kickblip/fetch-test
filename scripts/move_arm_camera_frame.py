#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from fetch_test.srv import MoveToPose, MoveToPoseResponse
import tf

def handle_move_to_pose(req):
    # Initialize moving flag
    moving = False
    
    # Create a response object
    res = MoveToPoseResponse()
    
    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    
    # Initialize PlanningScene
    planning_scene = PlanningSceneInterface("base_link")
    
    # Initialize TF listener
    listener = tf.TransformListener()

    # Wait for TF to initialize
    rospy.sleep(2)
    
    try:
        # Look up the transformation between base_link and camera_frame
        (trans, rot) = listener.lookupTransform('base_link', 'camera_frame', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to fetch transformation")
        res.success = False
        return res

    # Initialize the PoseStamped message in camera frame
    pose_in_camera_frame = req.target_pose
    
    # Transform pose from camera_frame to base_link
    pose_in_camera_frame.header.frame_id = 'camera_frame'
    pose_in_base_link = listener.transformPose('base_link', pose_in_camera_frame)

    if not moving:
        moving = True
        
        # Set the current timestamp
        pose_in_base_link.header.stamp = rospy.Time.now()
        
        # Move the robot arm to the target pose
        move_group.moveToPose(pose_in_base_link, "wrist_roll_link")
        result = move_group.get_move_action().get_result()
        
        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Moved to target pose!")
                res.success = True
            else:
                rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
                res.success = False
        else:
            rospy.logerr("MoveIt! failure no result returned.")
            res.success = False
        
        moving = False

    return res

if __name__ == '__main__':
    rospy.init_node("move_to_target_pose_service")

    # Initialize the service
    s = rospy.Service('move_to_pose_in_camera_frame', MoveToPose, handle_move_to_pose)

    rospy.spin()
