#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
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

# Initialize the ROS node
rospy.init_node('image_viewer', anonymous=True)

# Subscribe to the 'head_camera/rgb/image_raw' topic
image_topic = "/head_camera/rgb/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)

# Keep the program running
rospy.spin()

# Destroy OpenCV windows
cv2.destroyAllWindows()
