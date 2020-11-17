#!/usr/bin/python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("output_image",Image,queue_size=0)
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)
        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        output = self.process(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def process(self,cv_image):
        #TODO something
        return cv_image

if __name__ == '__main__':
    rospy.init_node('camera_cv', anonymous=True)
    ic = image_converter()
    rospy.spin()