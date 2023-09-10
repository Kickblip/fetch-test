#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import random

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

        # Set random position within some reasonable bounds (modify as needed)
        target_pose.position.x = random.uniform(0.1, 0.5)
        target_pose.position.y = random.uniform(-0.2, 0.2)
        target_pose.position.z = random.uniform(0.1, 0.5)

        # Set random orientation (quaternion values; modify as needed)
        target_pose.orientation.x = random.uniform(0, 1)
        target_pose.orientation.y = random.uniform(0, 1)
        target_pose.orientation.z = random.uniform(0, 1)
        target_pose.orientation.w = random.uniform(0, 1)

        # Normalize the quaternion (to make it a valid orientation)
        norm_factor = (target_pose.orientation.x ** 2 + 
                       target_pose.orientation.y ** 2 + 
                       target_pose.orientation.z ** 2 + 
                       target_pose.orientation.w ** 2) ** 0.5
        target_pose.orientation.x /= norm_factor
        target_pose.orientation.y /= norm_factor
        target_pose.orientation.z /= norm_factor
        target_pose.orientation.w /= norm_factor

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
