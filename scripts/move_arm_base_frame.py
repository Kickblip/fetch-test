#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from fetch_test.srv import MoveToPose, MoveToPoseResponse

def handle_move_to_pose(req):
    
    # Create a response object
    res = MoveToPoseResponse()
    
    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    
    # Initialize PlanningScene
    planning_scene = PlanningSceneInterface("base_link")
    
    # Initialize the PoseStamped message
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    
    # Set the current timestamp
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    
    # Set the target pose
    gripper_pose_stamped.pose = req.target_pose
    
    # Move the robot arm to the target pose
    move_group.moveToPose(gripper_pose_stamped, "wrist_roll_link")
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

    return res

if __name__ == '__main__':
    rospy.init_node("move_to_target_pose_service")

    # Initialize the service
    s = rospy.Service('move_to_pose', MoveToPose, handle_move_to_pose)

    rospy.spin()
