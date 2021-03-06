#!/usr/bin/python

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self, image_topic = "/thorvald_001/kinect2_camera/hd/image_color_rect"):
        self.image_pub = rospy.Publisher("opencv_image",Image,queue_size=0)
        self.green_detected_pub = rospy.Publisher("green_detected",String,queue_size=0)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        output,mask = self.green_mask(cv_image)


        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
            if np.sum(mask) > 1e6: #this threshold was chosen arbitrarily
                self.green_detected_pub.publish("TRUE")
            else:
                self.green_detected_pub.publish("FALSE")

        except CvBridgeError as e:
            print(e)
    
    def green_mask(self, cv_image):
        """
        masks out elements of the image that have a green value below 70.
        returns output and mask
        """
        #boundaries checking for green values between 70 and 255
        lower_green = np.array([0,70,0],dtype='uint8')
        upper_green = np.array([255,255,255],dtype='uint8')

        mask = cv2.inRange(cv_image,lower_green,upper_green)
        output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

        return output,mask
    

if __name__ == '__main__':
    ic = image_converter()
    rospy.init_node('vision_node', anonymous=True)
    rospy.spin()
