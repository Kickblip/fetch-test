#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
from moveit_msgs.msg import MoveItErrorCodes
import cv2


bridge = CvBridge()


class FollowTrajectoryClient(object):
    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient(
            "%s/follow_joint_trajectory" % name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()


class PointHeadClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()


def get_grasp_pose_from_ar_marker(marker_pose):
    # returns pose for now
    return marker_pose.pose


def move_robot(target_pose):
    global move_group
    global planning_scene

    # Go to the target pose
    result = move_group.moveToPose(target_pose, "gripper_link")
    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Robot moved to target pose!")
        else:
            rospy.logwarn("Failed to move to target pose!")
    else:
        rospy.logwarn("MoveIt! did not return any result.")

    move_group.get_move_action().cancel_all_goals()


def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)
    except CvBridgeError as e:
        print(e)
        return


def ar_marker_callback(marker_msg):
    for marker in marker_msg.markers:
        rospy.loginfo("AR marker detected with ID: %d!" % marker.id)
        grasp_pose = get_grasp_pose_from_ar_marker(marker)
        move_robot(grasp_pose)


if __name__ == "__main__":
    rospy.init_node("pick_and_place_demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    torso_action = FollowTrajectoryClient(
        "torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()

    # Raise the torso
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])

    # Lower the head
    rospy.loginfo("Lowering head...")
    head_action.look_at(1.0, 0.0, -0.5, "base_link")

    # Subscribe to the AR marker detection and camera topics
    image_topic = "/head_camera/rgb/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    ar_marker_topic = "/ar_pose_marker"
    rospy.Subscriber(ar_marker_topic, AlvarMarkers, ar_marker_callback)

    rospy.spin()

    cv2.destroyAllWindows()
