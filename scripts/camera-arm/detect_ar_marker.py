#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import tf
import cv2

# Initialize the CvBridge class
bridge = CvBridge()
moving = False
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


# Create move group interface for a fetch robot
move_group = MoveGroupInterface(
    "arm_with_torso", "base_link")

# Initialize PlanningScene
planning_scene = PlanningSceneInterface("base_link")

# Initialize TF listener
listener = tf.TransformListener()


def ar_marker_callback(marker_msg):
    global moving
    global move_group
    global planning_scene
    global listener

    for marker in marker_msg.markers:

        if (marker.id == 14) and (not moving):

            rospy.loginfo("AR marker detected!")
            moving = True

            # Use pose of the marker - taken in the camera frame
            gripper_pose_stamped = PoseStamped(
                pose=marker.pose.pose,
                header=Header(frame_id=marker.header.frame_id))

            # Transform the pose from head_camera_link to base_link
            try:
                pose_in_base_link = listener.transformPose(
                    'base_link', gripper_pose_stamped)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                continue

            # Move the robot arm to the target pose
            move_group.moveToPose(pose_in_base_link, "wrist_roll_link")
            result = move_group.get_move_action().get_result()

            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Moved to target pose!")
                    moving = False
                else:
                    rospy.logerr("Arm goal in state: %s",
                                 move_group.get_move_action().get_state())
                    moving = False
            else:
                rospy.logerr("MoveIt failure no result returned.")


# Initialize the ROS node
rospy.init_node('ar_marker_detector', anonymous=True)

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
