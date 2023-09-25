#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Initialize the CvBridge class
bridge = CvBridge()

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
    for marker in marker_msg.markers:
        print(f"AR Tag with ID {marker.id} detected.")

        if marker.id == 1234:
            print("Specific AR Tag with ID 1234 detected.")
            # Move robot arm

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
