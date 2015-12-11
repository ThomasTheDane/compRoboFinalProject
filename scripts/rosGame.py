#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3
from neatoLocation import MarkerProcessor
from ar_pose.msg import ARMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from tf import TransformListener, TransformBroadcaster
from copy import deepcopy
from math import sin, cos, pi, atan2, fabs
import rospkg

""" This code implements a ceiling-marker based localization system.
    The code is slightly modified based on the number/kinds of markers 
    we've decided to use.
    The core of the code is filling out the marker_locators
    which allow for the specification of the position and orientation
    of the markers on the ceiling of the room """

class Neatobot:
    """One player/robot"""

    def __init__(self):
        """ Initialize the ball tracker """
        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        cv2.namedWindow('video_window')
        
        rospy.init_node('neato_bot')
        self.cv_image = None                        # the latest image from the camera    
        self.points = None
        self.pixel = None
        self.D= 0
        self.K= 0
        self.cx = 0
        self.cy = 0
        self.cz = 0
        self.theta = 0
        self.swap_new_coord = None
        self.coins = []
        self.coinPixels = []
        self.coinVisible = True
 
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        rospy.Subscriber("STAR_pose_continuous",PoseStamped, self.processLocation)
        rospy.Subscriber("camera/camera_info", CameraInfo, self.get_camerainfo)
        # self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
    def processLocation(self,msg):
        self.cx = msg.pose.position.x
        self.cy = msg.pose.position.y
        self.cz = msg.pose.position.z
        euler_angles = euler_from_quaternion((msg.pose.orientation.x,
                                                  msg.pose.orientation.y,
                                                  msg.pose.orientation.z,
                                                  msg.pose.orientation.w))
        print "theta from processLocation: ", euler_angles
        self.theta = -euler_angles[2]
        #TODO
        #print self.cx, self.cy, self.cz

    # def processEulerAngle(self, vec3):
        # self.theta = -vec3.z
        # print "theta from subcrib: ", self.theta
        # TODO
        # theta should be negated

    def transformWorldToCamera(self, aCoin):
        self.coinPixels = []
        aCoinPixels = []

        for aCoord in aCoin:

            rotat = rotation_matrix(self.theta,[0,0,1])
            trans = np.matrix([[1,0,0,-self.cx],[0,1,0,-self.cy],[0,0,1,-self.cz],[0,0,0,1]])

            new_coord = np.dot(rotat, np.dot(trans, aCoord))
            

            aCoinPixels.append(self.pixelate(new_coord[:3]))

        self.coinPixels.append(aCoinPixels)
        

    def pixelate(self, new_coord):

    	# for i in range(new_coord.shape[0]): 
    	# 	b[i][0]=-new_coord[i][1]
    	# 	b[i][1]=-new_coord[i][2]
    	# 	b[i][2]= new_coord[i][0]
    		
        new_coord = [-new_coord[1,0], -new_coord[2,0], new_coord[0,0]]
        if new_coord[2] < 0:
        	self.coinVisible = False
        else:
            self.coinVisible = True

        new_coord = np.array([new_coord])
        # points = cv2.projectPoints(new_coord, (0,0,0), (0,0,0), self.K, self.D)
        # self.points = points[0]
        
        p = self.K.dot(new_coord.T)
        return p / p[2]
        # print "self.pixel", self.pixel
    
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

    def get_camerainfo(self,msg):
        self.D = msg.D
        self.K = np.resize(msg.K, (3,3))

    def process_image(self,msg):

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.coinPixels:
            for aCoin in self.coinPixels:
                toCoord = []
                print "aCoin: ", aCoin
                if self.coinVisible:                    
                    #img = self.warpAndCombine(cv2.imread("coin.png", 1), , self.cv_image)
                    for aPixel in aCoin:
                        toCoord.append([aPixel[0], aPixel[1]])
                        cv2.circle(self.cv_image,(int(aPixel[0]), int(aPixel[1])), 5, (0, 0, 255))
                    #print(cv2.imread(self.rospack.get_path('finalproject') + "/scripts/coin.png", 1))
                    self.cv_image = self.warpAndCombine(cv2.imread(self.rospack.get_path('finalproject') + "/scripts/coin.png", 1), toCoord, self.cv_image)

        cv2.imshow('video_window', self.cv_image)
        cv2.waitKey(10)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.theta!=0:
                coin_coord = np.array([[[1], [0], [0], [1]],[[1], [0], [.2], [1]],[[1], [.2], [0], [1]],[[1], [.2], [.2], [1]]],dtype='float32')
#                coin_coord = np.array([[[1], [.2], [.2], [1]],[[1], [0], [.2], [1]],[[1], [.2], [0], [1]],[[1], [0], [0], [1]]],dtype='float32')

                self.transformWorldToCamera(coin_coord)
            r.sleep()

if __name__ == '__main__':
    nb = Neatobot()
    nb.run()
