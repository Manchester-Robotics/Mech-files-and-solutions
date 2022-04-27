#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

record = True # flag to save samples in diferent stages of the experiment in a video

# Reject outliers by keeping the biggest area detected
def keep_bigest_blob(keypoints):

    aux_size = 0
    best_blob = []
    id = -1

    for i in range (0, len(keypoints)):

        if keypoints[i]:
            if keypoints[i].size > aux_size:
                best_blob = keypoints[i]
                id = i
                aux_size = keypoints[i].size

    return best_blob, id


class ros_main():

    def __init__(self):

        # Red channel needs two diferent areas in the hsv color space
        self.lower_limits = [160, 49, 20]  # r
        self.upper_limits = [179, 255, 255]  # r

        self.lower_limits2 = [0, 49, 20]  # r
        self.upper_limits2 = [10, 255, 255]  # r

        # Note: Very narrow values, not very good for a general case
        self.lower_limits3 = [26, 12, 20]  # g
        self.upper_limits3 = [100, 255, 255]  # g

        # configuration of the detector
        params = cv2.SimpleBlobDetector_Params()

        params.filterByArea = True
        params.minArea = 2000
        params.maxArea = 10000000 # hotfix meaning that the blob can be as big as we want, defaults to 5k pixels
        params.filterByInertia = False
        params.filterByConvexity = False
        params.filterByCircularity = True
        params.minCircularity = 0.35

        # create the detection object
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.potential_keypoints = []

        self.vel_msg = Twist()

        if record:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out1 = cv2.VideoWriter('/home/puzzlebot/catkin_ws/src/tf_rec/src/output1.avi', fourcc, 20.0, (1280, 720))
            self.out2 = cv2.VideoWriter('/home/puzzlebot/catkin_ws/src/tf_rec/src/output2.avi', fourcc, 20.0, (1280, 720))
            self.out3 = cv2.VideoWriter('/home/puzzlebot/catkin_ws/src/tf_rec/src/output3.avi', fourcc, 20.0, (1280, 720))
            self.out4 = cv2.VideoWriter('/home/puzzlebot/catkin_ws/src/tf_rec/src/output4.avi', fourcc, 20.0, (1280, 720))

        rospy.init_node("tf_rec")
        self.rate = rospy.Rate(30)
        self.img = np.array([]) # image buffer, not very useful in this experiment

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.camera_callback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 3)

    def camera_callback(self, msg):

        self.img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def run(self):

        while not rospy.is_shutdown():
            # we only study the newest image available
            if not self.img.size == 0:

                # store current image and clean buffer
                img = self.img
                self.img = np.array([])

                # NOTE: Code could be improved by filtering both red and green at the same time
                # BEG of red filtering
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                # we make a mask of 1 and 0 depending in the values of the images are in range or note
                mask1 = cv2.inRange(hsv, np.array(self.lower_limits), np.array(self.upper_limits))
                # apply logical and between image and mask
                a1 = cv2.bitwise_and(img, img, mask=mask1)
                mask2 = cv2.inRange(hsv, np.array(self.lower_limits2), np.array(self.upper_limits2))
                a2 = cv2.bitwise_and(img, img, mask=mask2)

                # add both masks to consider all the red present in hsv, the resulting image contains only redish patches
                result = cv2.bitwise_or(a1, a2)

                if record:
                    # write a frame into a video file
                    self.out1.write(result)

                # map to grayscale and threshold it to get a binary image
                result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                _ , result = cv2.threshold(result, 0, 255, cv2.THRESH_BINARY_INV) # Note: This thresholding can be tuned to reject some noise

                result_aux = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR) # reformating to save image in the video
                if record:
                    self.out2.write(result_aux)

                # apply erode - dilate to remove nosie and avoid having holes in the shapes
                kernel = np.ones((4, 4), np.uint8)

                result = cv2.dilate(result, kernel, iterations=3)
                result = cv2.erode(result, kernel, iterations=3)

                result_aux = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
                if record:
                    self.out3.write(result_aux)

                # detect blobs
                keypoints = self.detector.detect(result)

                # store biggest blob as a candidate
                keypoint, id = keep_bigest_blob(keypoints)
                self.potential_keypoints.append(keypoint)

                # END of red filtering

                ## BEG Green filtering: no comments cause we use the same tools that we saw before

                mask = cv2.inRange(hsv, np.array(self.lower_limits3), np.array(self.upper_limits3))
                result = cv2.bitwise_and(img, img, mask=mask)

                if record:
                    self.out1.write(result)

                result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                _ , result = cv2.threshold(result, 0, 255, cv2.THRESH_BINARY_INV)

                result_aux = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
                if record:
                    self.out2.write(result_aux)

                kernel = np.ones((4, 4), np.uint8)

                result = cv2.dilate(result, kernel, iterations=3)
                result = cv2.erode(result, kernel, iterations=3)

                keypoints = self.detector.detect(result)

                keypoint, id = keep_bigest_blob(keypoints)

                self.potential_keypoints.append(keypoint)

                # END of green filtering

                ## Debug
                try:

                    # among all the candidates pick the biggest one and clear the list
                    keypoint, id = keep_bigest_blob(self.potential_keypoints)
                    self.potential_keypoints = []

                    # draw keypoints in an image
                    blank = np.zeros((1, 1))
                    blobs = cv2.drawKeypoints(img, (keypoint,), blank, (0, 0, 0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                    # PIECE OF CODE THAT CAN BE USED TO DISPLAY AN IMAGE
                    # cv2.imshow("Blobs Using Area", blobs)
                    # cv2.waitKey(1)
                    res = blobs
                    # move if green, stop if red within a range
                    if keypoint.size > 175 and id == 0:
                        self.vel_msg.linear.x = 0
                    elif id == 1:
                        self.vel_msg.linear.x = 0.2
                    else:
                        self.vel_msg.linear.x = 0

                except:
                    # if we don't see anything stop
                    res=img
                    self.vel_msg.linear.x = 0.0

                if record:
                    self.out4.write(res)
                # send velocity comand
                self.vel_pub.publish(self.vel_msg)

        if record:
            # close video objects
            self.out1.release()
            self.out2.release()
            self.out3.release()
            self.out4.release()

if __name__ == '__main__':

        t = ros_main()
        t.run()




