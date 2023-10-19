import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class RotateRobot:

    def __init__(self):
        rospy.init_node('rotate_180_degrees', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0.0
        self.start_yaw = None

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def rotate_180_degrees(self):
        rospy.sleep(1)  # Give subscribers time to get initial data
        self.start_yaw = self.current_yaw

        rotate_cmd = Twist()
        rotate_cmd.angular.z = 1.0

        while not rospy.is_shutdown():
            delta_yaw = abs(self.current_yaw - self.start_yaw)
            if delta_yaw >= 3.14:  # If rotation is roughly 180 degrees or more
                break
            self.pub.publish(rotate_cmd)
            rospy.sleep(0.01)

        # Stop the robot
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)


if __name__ == '__main__':
    try:
        robot = RotateRobot()
        robot.rotate_180_degrees()
    except rospy.ROSInterruptException:
        pass
