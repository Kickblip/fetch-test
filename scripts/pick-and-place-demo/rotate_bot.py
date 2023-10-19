#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def continuous_rotate(duration):
    # Initialize the node
    rospy.init_node('continuous_rotate', anonymous=True)

    # Create a publisher to the cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Give it a moment to initialize
    rospy.sleep(1)

    # Create a Twist message
    rotate_cmd = Twist()

    # Set the angular velocity to a value that will rotate the robot. Adjust this value based on your robot's capabilities
    # For example, rotate at 1 rad/s. Adjust this value as needed.
    rotate_cmd.angular.z = 1.0

    # Publish the rotation command for the desired duration
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        pub.publish(rotate_cmd)
        # This will publish the command at 10Hz, adjust as needed
        rospy.sleep(0.1)

    # Stop the robot by publishing a zero Twist message
    stop_cmd = Twist()
    pub.publish(stop_cmd)


if __name__ == '__main__':
    try:
        # Rotate for 5 seconds as an example. Change the duration as needed.
        continuous_rotate(5)
    except rospy.ROSInterruptException:
        pass
