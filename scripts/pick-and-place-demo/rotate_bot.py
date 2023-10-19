#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time


def rotate_180_degrees():
    # Initialize the node
    rospy.init_node('rotate_180_degrees', anonymous=True)

    # Create a publisher to the cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Give it a moment to initialize
    rospy.sleep(1)

    # Create a Twist message
    rotate_cmd = Twist()

    # Set the angular velocity to a value that will rotate the robot. Adjust this value based on your robot's capabilities
    # Compute the rotation time for 180 degrees based on the given angular velocity
    rotation_time = 3.14 / rotate_cmd.angular.z

    # Increase the rotation time by 20% as a correction factor
    rotation_time *= 1.2

    # Publish the rotation command
    pub.publish(rotate_cmd)

    # Wait for the rotation to complete
    time.sleep(rotation_time)

    # Stop the robot by publishing a zero Twist message
    stop_cmd = Twist()
    pub.publish(stop_cmd)


if __name__ == '__main__':
    try:
        rotate_180_degrees()
    except rospy.ROSInterruptException:
        pass
