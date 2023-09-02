#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def move_to_pose(target_pose):
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_to_pose', anonymous=True)
    
    # Initialize the MoveGroup for the arm
    arm_group = moveit_commander.MoveGroupCommander("arm")
    
    # Set the target pose
    arm_group.set_pose_target(target_pose)
    
    # Plan the trajectory
    plan = arm_group.plan()
    
    # Execute the trajectory
    arm_group.execute(plan, wait=True)
    
    # Stop and delete all remaining goals (not strictly necessary, but good practice)
    arm_group.stop()
    arm_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        # Create a Pose object with the target coordinates and orientation
        target_pose = Pose()
        target_pose.position.x = 0.5
        target_pose.position.y = 0.0
        target_pose.position.z = 0.5
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0
        
        move_to_pose(target_pose)
        
    except rospy.ROSInterruptException:
        pass
