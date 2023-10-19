#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def rotate_180_degrees():
    # Initialize the ROS node
    rospy.init_node('rotate_robot_180', anonymous=True)

    # Publisher for cmd_vel
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rotation_duration = 1.0
    rate = rospy.Rate(10)
    end_time = rospy.Time.now() + rospy.Duration(rotation_duration)

    print("Starting rotation...")  # Debugging statement

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        twist_msg = Twist()
        twist_msg.angular.z = 3.14
        pub.publish(twist_msg)
        print("Publishing rotation command...")  # Debugging statement
        rate.sleep()

    # Stop the robot after rotation
    print("Stopping rotation...")  # Debugging statement
    pub.publish(Twist())


if __name__ == '__main__':
    try:
        rotate_180_degrees()
    except rospy.ROSInterruptException:
        print("Rotation interrupted.")  # Debugging statement
