
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import cv2


bridge = CvBridge()


rospy.init_node('torso_head_mover', anonymous=True)


move_group = MoveGroupInterface("torso_and_head", "base_link")
planning_scene = PlanningSceneInterface("base_link")


def image_callback(img_msg):
    global bridge
    try:

        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)


def move_robot():

    joints = ["torso_lift_joint", "head_tilt_joint"]

    joint_values = [0.3, -0.5]

    move_group.moveToJointPosition(joints, joint_values, wait=True)

    result = move_group.get_move_action().get_result()
    if result:
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Robot moved to target joint values!")
        else:
            rospy.logwarn("Failed to move to target joint values!")
    else:
        rospy.logwarn("MoveIt! did not return any result.")

    move_group.get_move_action().cancel_all_goals()


image_topic = "/head_camera/rgb/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)


rospy.sleep(1)


move_robot()


rospy.spin()


cv2.destroyAllWindows()
