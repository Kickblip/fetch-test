#!/usr/bin/env python

import rospy
from fetch_test.srv import MoveToPose
from geometry_msgs.msg import Pose

def move_arm_to_poses():
    # Initialize the node
    rospy.init_node('pose_publisher_node', anonymous=True)

    # Wait for the move_to_pose service to become available
    rospy.wait_for_service('move_to_pose')
    
    # Create a ServiceProxy object
    move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)

    # List of target poses
    poses = [
        Pose(position=Point(x=0.4, y=0.2, z=0.6), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        Pose(position=Point(x=0.5, y=0.2, z=0.6), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        # Add more poses here
    ]

    for target_pose in poses:
        try:
            # Call the move_to_pose service and wait for it to complete
            response = move_to_pose(target_pose)

            # Check if the move was successful (replace with your actual condition)
            if response.success:
                rospy.loginfo("Successfully moved to pose: %s", target_pose)
            else:
                rospy.logwarn("Failed to move to pose: %s", target_pose)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        # Add some delay between poses (optional)
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        move_arm_to_poses()
    except rospy.ROSInterruptException:
        pass
