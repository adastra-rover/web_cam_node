import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class WebCamPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('web_cam_publisher')
        # Create a CvBridge to convert between ROS messages and OpenCV images
        self.bridge = CvBridge()
        # Create a publisher for the image topic
        self.image_pub = rospy.Publisher('web_cam_image', Image, queue_size=10)
        # Start the web cam
        self.web_cam = cv2.VideoCapture(0)

    def start(self):
        # Run the publisher at 30 Hz
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # Capture a frame from the web cam
            success, frame = self.web_cam.read()
            # Check if the frame was successfully captured
            if not success:
                continue
            # Convert the frame to a ROS message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # Publish the message
            self.image_pub.publish(msg)
            # Sleep for the rest of the time until the next iteration
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = WebCamPublisher()
        publisher.start()
    except rospy.ROSInterruptException:
        pass
