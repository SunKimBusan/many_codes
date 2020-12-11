#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class converter:

    def __init__(self):

        self.br = CvBridge()
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback,  queue_size = 1)
        self.pub = rospy.Publisher('/camera/color/image_raw', Image,queue_size=10)

    def callback(self, ros_data):

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        self.pub.publish(self.br.cv2_to_imgmsg(image_np, encoding="bgr8"))

if __name__ == '__main__':
    rospy.init_node('CompressedImageToImage', anonymous=True)
    converter = converter()
    rospy.spin()
