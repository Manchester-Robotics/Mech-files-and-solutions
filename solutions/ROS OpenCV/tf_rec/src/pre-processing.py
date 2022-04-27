#!/usr/bin/env python

from ast import Pass
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image


class image_processor:
    def __init__(self):
        self.frameWidth = 640
        self.frameHeight = 320

        self.image = None
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 

        self.image_pub = rospy.Publisher('/processed',Image, queue_size=10)
    def image_callback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def preprocess(self,image):
        rot = cv2.rotate(image,cv2.ROTATE_180)
        resized = cv2.resize(rot,(self.frameWidth, self.frameHeight))
        src_gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(src_gray, (19, 19), 0)
        return blurred

    def run(self):
        if self.image is None:
            return
        processed = self.preprocess(self.image)
        ros_img = self.bridge.cv2_to_imgmsg(processed)
        self.image_pub.publish(ros_img)

if __name__ == "__main__":
    image_proc = image_processor()
    rospy.init_node("image_processor")
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            image_proc.run()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass