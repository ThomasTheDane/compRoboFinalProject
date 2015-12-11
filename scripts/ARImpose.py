#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ARStuff(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self):
        """ Initialize the ball tracker """
        rospy.init_node('AR')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        cv2.namedWindow('video_window')
        cv2.waitKey(5)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.circle(self.cv_image, (50, 50), 5, (255, 255, 255))
        img = self.warpAndCombine(cv2.imread("coin.png", 1), [[0,50], [200,0], [0,100], [200, 200]], self.cv_image)

        cv2.imshow('video_window', img)
        cv2.waitKey(10)

    def warpAndCombine(self, sourceImage, destinationPoints, destinationImage):
        rows, cols, _ = sourceImage.shape
        sourcePoints = np.float32([[0,0], [cols, 0],[0, rows], [cols, rows]])
        destinationPoints = np.float32(destinationPoints)
        M = cv2.getPerspectiveTransform(sourcePoints, destinationPoints)

        newWidth = max([destinationPoints[1][0], destinationPoints[3][0]])
        newHeight = max([destinationPoints[2][1], destinationPoints[3][1]])
        warpedImage = cv2.warpPerspective(sourceImage, M, (newWidth, newHeight))
        
        #if warpedImage[0,0][0] == 0 and warpedImage[0,0][1] == 0 and warpedImage[0,0][2] == 0:
        
        finalImage = destinationImage

        # Load two images

        finalImage = self.overlayImages(finalImage, warpedImage)

        return finalImage

# 
    def overlayImages(self, img1, img2):

        # I want to put logo on top-left corner, So I create a ROI
        rows,cols,channels = img2.shape
        roi = img1[0:rows, 0:cols ]

        # Now create a mask of logo and create its inverse mask also
        img2gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)

        # Now black-out the area of logo in ROI
        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)

        # Take only region of logo from logo image.
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)

        # Put logo in ROI and modify the main image
        dst = cv2.add(img1_bg,img2_fg)
        img1[0:rows, 0:cols ] = dst
        return img1

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        #warpImage(img, [,])

        while not rospy.is_shutdown():
         # start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    node = ARStuff()
    node.run()