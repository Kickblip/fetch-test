#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def turn_180_degrees():
    # Initialize the node
    rospy.init_node('direct_turn_180_degrees', anonymous=True)

    # Create a publisher for the base_controller/command topic
    pub = rospy.Publisher('base_controller/command', Twist, queue_size=10)

    # Create a Twist message and set linear.x = 0 since we're only turning
    twist = Twist()
    twist.linear.x = 0.0

    # Set the angular.z velocity for turning
    turning_velocity = 1.0  # rad/s
    twist.angular.z = turning_velocity

    # Calculate the time to turn 180 degrees
    turn_duration = 3.14159 / turning_velocity  # pi radians is 180 degrees

    # Start turning
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(turn_duration) and not rospy.is_shutdown():
        pub.publish(twist)

    # Stop the robot after turning
    twist.angular.z = 0.0
    pub.publish(twist)


if __name__ == '__main__':
    try:
        turn_180_degrees()
    except rospy.ROSInterruptException:
        pass
