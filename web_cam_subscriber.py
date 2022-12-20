import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class WebCamSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('web_cam_subscriber')
        # Create a CvBridge to convert between ROS messages and OpenCV images
        self.bridge = CvBridge()
        # Create a subscriber for the image topic
        self.image_sub = rospy.Subscriber('web_cam_image', Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert the ROS message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        else:
            # Do something with the image here, like display it
            cv2.imshow('web cam image', frame)
            cv2.waitKey(1)

if __name__ == '__main__':
    try:
        subscriber = WebCamSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
