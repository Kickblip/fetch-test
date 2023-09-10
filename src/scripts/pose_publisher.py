#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def pose_publisher():
    # Initialize the node
    rospy.init_node('pose_publisher_node', anonymous=True)
    
    # Create a Publisher object, specify the topic name, message type, and queue size
    pub = rospy.Publisher('/target_pose', Pose, queue_size=10)

    # Set the rate of publishing (in Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create a Pose message object
        target_pose = Pose()
        
        # Set the position (example values)
        target_pose.position.x = 0.4
        target_pose.position.y = 0.2
        target_pose.position.z = 0.6

        # Set the orientation (example values, in quaternion)
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0

        # Publish the Pose message
        pub.publish(target_pose)

        # Log the publishing
        rospy.loginfo("Published target pose: %s", target_pose)

        # Sleep to maintain the rate
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
