#!/usr/bin/env python

import rospy
from fetch_test.srv import MoveToPose
from geometry_msgs.msg import Pose, Point, Quaternion

def move_arm_to_poses():
    # Initialize the node
    rospy.init_node('pose_publisher_node', anonymous=True)

    # Wait for the move_to_pose service to become available
    rospy.wait_for_service('move_to_pose')
    
    # Create a ServiceProxy object
    move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)

    # List of target poses
    poses = [
        Pose(position=Point(x=0.35, y=0.25, z=0.65), orientation=Quaternion(x=0.1, y=0.0, z=0.0, w=1.0)),
        Pose(position=Point(x=0.52, y=0.18, z=0.59), orientation=Quaternion(x=0.0, y=0.1, z=0.0, w=1.0)),
        Pose(position=Point(x=0.29, y=0.12, z=0.55), orientation=Quaternion(x=0.0, y=0.0, z=0.1, w=1.0)),
        Pose(position=Point(x=0.48, y=0.32, z=0.73), orientation=Quaternion(x=0.1, y=0.1, z=0.0, w=1.0)),
        Pose(position=Point(x=0.63, y=0.24, z=0.81), orientation=Quaternion(x=0.0, y=0.1, z=0.1, w=1.0)),
        Pose(position=Point(x=0.71, y=0.08, z=0.64), orientation=Quaternion(x=0.1, y=0.0, z=0.1, w=1.0)),
        Pose(position=Point(x=0.39, y=0.42, z=0.75), orientation=Quaternion(x=0.1, y=0.1, z=0.1, w=1.0)),
        Pose(position=Point(x=0.34, y=0.27, z=0.62), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        Pose(position=Point(x=0.52, y=0.09, z=0.53), orientation=Quaternion(x=0.1, y=0.0, z=0.0, w=1.0)),
        Pose(position=Point(x=0.58, y=0.35, z=0.63), orientation=Quaternion(x=0.0, y=0.1, z=0.0, w=1.0))
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
