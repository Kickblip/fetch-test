#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def move_forward():
    rospy.init_node('move_forward_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    forward_cmd = Twist()
    forward_cmd.linear.x = 0.5  # setting x-axis linear velocity to move the robot forward
    forward_cmd.angular.z = 0.0  # no angular velocity

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pub.publish(forward_cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
