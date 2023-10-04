#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import cv2

# Initialize the CvBridge class
bridge = CvBridge()
moving = False

# Initialize the ROS node
rospy.init_node('ar_marker_detector', anonymous=True)

# Initialize move_group interface
move_group = MoveGroupInterface("arm_with_torso", "base_link")
planning_scene = PlanningSceneInterface("base_link")

# Callback function for the subscribed topic


def image_callback(img_msg):
    global bridge
    try:
        # Convert ROS image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Display the OpenCV image
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)

# Callback function for AR marker detection


def ar_marker_callback(marker_msg):
    global moving

    for marker in marker_msg.markers:
        if (not moving):
            rospy.loginfo("AR marker detected!")
            moving = True

            # Use pose of the marker
            gripper_pose_stamped = PoseStamped(
                pose=marker.pose.pose,
                header=Header(frame_id=marker.header.frame_id))

            move_robot(gripper_pose_stamped)


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


# Subscribe to the 'head_camera/rgb/image_raw' topic
image_topic = "/head_camera/rgb/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)

# Subscribe to the 'ar_pose_marker' topic
ar_marker_topic = "/ar_pose_marker"
rospy.Subscriber(ar_marker_topic, AlvarMarkers, ar_marker_callback)

# Keep the program running
rospy.spin()

# Destroy OpenCV windows
cv2.destroyAllWindows()
