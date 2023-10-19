#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def rotate_180_degrees():
    # Initialize the ROS node
    rospy.init_node('rotate_robot_180', anonymous=True)

    # Publisher for cmd_vel
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Assuming it takes 2 seconds for a full 360 degree rotation at maximum speed
    # Adjust this time based on your robot's speed
    rotation_duration = 1.0  # Time it takes to rotate 180 degrees
    rate = rospy.Rate(10)  # 10Hz
    end_time = rospy.Time.now() + rospy.Duration(rotation_duration)

    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        twist_msg = Twist()
        # Assuming pi rad/sec is the maximum speed. Adjust as necessary.
        twist_msg.angular.z = pi
        pub.publish(twist_msg)
        rate.sleep()

    # Stop the robot after rotation
    pub.publish(Twist())


if __name__ == '__main__':
    try:
        rotate_180_degrees()
    except rospy.ROSInterruptException:
        pass
