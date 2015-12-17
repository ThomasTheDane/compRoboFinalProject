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
from std_msgs.msg import Header, Int32
from sensor_msgs.msg import CameraInfo
from tf import TransformListener, TransformBroadcaster
from copy import deepcopy
from math import sin, cos, pi, atan2, fabs
import rospkg
import math

""" 
This class gets the location of the robot in world, transforms into camera, and projects the 
objects that exist in the world coordinate systems onto a screen. 

By calling neatoLocation_revised.py, a slightly modified version of STAR_pose_continuous
in the my_pf ROS pkg, the function can subscribe to the node STAR_pose_continuous which publishes
robot's location in the world coordinate which is a ceiling-marker defined coordinate system
"""

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
        self.K= None
        self.cx = 0
        self.cy = 0
        self.cz = 0
        self.theta = 0
        self.swap_new_coord = None
        self.coins = []
        self.coinPixels = []
        self.coinVisible = True
        self.score=0
        self.coinInWorld = [] # needs to be initialized 

        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        rospy.Subscriber("STAR_pose_continuous",PoseStamped, self.processLocation)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.get_camerainfo)
        self.score_pub = rospy.Publisher("score", Int32, queue_size=1)
        
        """
        def processLocation(self,msg):
        A callBack function for STAR_pose_continuous which saves the location of the robot
        in the world coordinate system, calculates and save the angle of robot's Header
        """
    def calculateScore(self, x,y):
        padding = 0.2
        for i, coin in enumerate(self.coinInWorld):
            if abs(x-coin[0]) < padding and abs(y-coin[1]) < padding:
                self.score+=1
                self.coinInWorld.pop(i)
                self.score_pub.publish(Int32(self.score))

    def processLocation(self,msg):
        #from the STAR_pose_continuous, get the location of robot in world
        self.cx = msg.pose.position.x
        self.cy = msg.pose.position.y
        self.cz = msg.pose.position.z
        #calculate the angle of the robot based on the world coordinate system.
        #facing towards the classroom whiteboard is the x axis.
        euler_angles = euler_from_quaternion((msg.pose.orientation.x,
                                                  msg.pose.orientation.y,
                                                  msg.pose.orientation.z,
                                                  msg.pose.orientation.w))
        print "theta from processLocation: ", euler_angles
        self.theta = -euler_angles[2]

        self.calculateScore(self.cx, self.cy)

        """
        def transformWorldToCamera(self, aCoin):
        A function that transforms the coordinate systems, from world coordinate system to 
        a camera coordinate system.
        """
    def transformWorldToCamera(self, aCoin):
        #initialize the array which has elements of coins
        self.coinPixels = []
        #initialize the camera coordinate points for each coin image
        aCoinPixels = []

        for aCoord in aCoin:
            #rotational matrix, rotate by self.theta, along the z-axis
            rotat = rotation_matrix(self.theta,[0,0,1])
            #translation matrix which maps the location of a robot(in world) to camera coordinate origin
            trans = np.matrix([[1,0,0,-self.cx],[0,1,0,-self.cy],[0,0,1,-self.cz],[0,0,0,1]])
            #this calculates the points of the aCoin in camera coordinate system
            new_coord = np.dot(rotat, np.dot(trans, aCoord))
            #calls the pixelate function which maps the camera coordinate points into 2D screen
            aCoinPixels.append(self.pixelate(new_coord[:3]))
        #append to the array which takes an array for each coin
        self.coinPixels.append(aCoinPixels)

        
        """
        pixelate(self, new_coord):
        This function takes the points from the camera's perspective in a camera coordinate onto 
        a 640*480 screen.
        """
    def pixelate(self, new_coord):
        #swap the coordinates so that the depth into the image == x
        new_coord = [-new_coord[1,0], -new_coord[2,0], new_coord[0,0]]
        #if behind, don't show the image
        if new_coord[2] < 0:
            self.coinVisible = False
        else:
            self.coinVisible = True
        #instead of using projectPoints, we just use our own function
        #project points using the camera matrix
        new_coord = np.array([new_coord])
        p = self.K.dot(new_coord.T)
        return p / p[2]
        
        """
        rotatePoints(self,points,theta):
        This function takes the coordinate points in world and rotate it by theta. 
        The points should be in array of numbers
        """
    def rotatePoints(self,points,theta):
        #calculate the midpoint of an image
        midline= [1.1,0]
        # midline = [points[0][0][0] + ((points[1][0][0] - points[0][0][0]) / 2.0), points[0][1][0] + ((points[1][1][0] - points[0][1][0]) / 2.0)]
        
        #TODO
        #define the imagewidth, and will be changed to non-hardcoded value later
        imageWidth = 0.2
        #based on how we defined the points in the world coordinate for an image, 
        #this rotation will make the two side corner point pair rotate the a same amount 
        points[0][0] = midline[0] + (math.cos(theta)*(imageWidth*math.sqrt(2)/2))
        points[0][1] = midline[1] + (math.sin(theta)*(imageWidth*math.sqrt(2)/2))
        points[1][0] = midline[0] + (math.cos(theta)*(imageWidth*math.sqrt(2)/2))
        points[1][1] = midline[1] + (math.sin(theta)*(imageWidth*math.sqrt(2)/2))
        points[2][0] = midline[0] + (math.cos(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
        points[2][1] = midline[1] + (math.sin(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
        points[3][0] = midline[0] + (math.cos(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
        points[3][1] = midline[1] + (math.sin(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
            
        return points

        """
        warpAndCombine(self, sourceImage, destinationPoints, destinationImage):
        This function uses warpPerspective to get the matrix that transforms the images into
        the four corners of the image that is changing based on the location of a robot, and
        gets the warpedImage using the matrix.

        This function takes 
        sourceImage: the path of the image file,
        destinationPoints: location of each four corners of the image in the camera 
        system, 
        destinationImage: an opencv image that an object will be overlayed on.
        
        """
    def warpAndCombine(self, sourceImage, destinationPoints, destinationImage):
        rows, cols, _ = sourceImage.shape
        sourcePoints = np.float32([[0,0], [cols, 0],[0, rows], [cols, rows]])
        destinationPoints = np.float32(destinationPoints)
        
        M = cv2.getPerspectiveTransform(sourcePoints, destinationPoints)

        newWidth = max(destinationPoints[0][0],destinationPoints[1][0],destinationPoints[2][0], destinationPoints[3][0])
        newHeight = max(destinationPoints[0][1],destinationPoints[1][1],destinationPoints[2][1], destinationPoints[3][1])
        # print "newWidth is ", newWidth, "newHeight is ", newHeight

        finalImage = destinationImage

        if newWidth < 0 or newHeight < 0:
            return finalImage

        warpedImage = cv2.warpPerspective(sourceImage, M, (newWidth, newHeight))
        finalImage = self.overlayImages(finalImage, warpedImage)

        return finalImage

        """
        overlayImages(self, img1, img2):
        
        
        """
    def overlayImages(self, img1, img2):

        img2 = img2[0:img1.shape[0],0:img1.shape[1],:]
        

        rows,cols,channels = img2.shape
        roi = img1[0:rows, 0:cols ]
        # cv2.imshow('roi', roi)
        # cv2.waitKey(5)

        # Now create a mask of logo and create its inverse mask also
        img2gray = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
    
        mask = mask[0:roi.shape[0], 0:roi.shape[1]]
        mask_inv = cv2.bitwise_not(mask)
        
        # Now black-out the area of logo in ROI
        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        
       
        # Take only region of logo from logo image.
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
        
        # Put logo in ROI and modify the main image
        dst = cv2.add(img1_bg,img2_fg)
        img1[0:rows, 0:cols ] = dst
        return img1

        """
        def get_camerainfo(self,msg):
        this gets the camera info and save it in the init variable.
        
        """
    def get_camerainfo(self,msg):
        self.D = msg.D
        self.K = np.resize(msg.K, (3,3))

        """
        def process_image(self,msg):
        
        
        """
    def process_image(self,msg):

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        """
        def render_coin(self):
        
        
        """
    def render_coin(self):
        cur_image = np.array(self.cv_image).copy()

        if self.coinPixels:
            for aCoin in self.coinPixels:
                toCoord = []
                # print "aCoin: ", aCoin
                if self.coinVisible:                    
                    #img = self.warpAndCombine(cv2.imread("coin.png", 1), , self.cv_image)
                    for aPixel in aCoin:
                        toCoord.append([aPixel[0], aPixel[1]])
                        cv2.circle(self.cv_image,(int(aPixel[0]), int(aPixel[1])), 5, (0, 0, 255))
                    #print(cv2.imread(self.rospack.get_path('finalproject') + "/scripts/coin.png", 1))
                    # self.cv_image = self.warpAndCombine(cv2.imread(self.rospack.get_path('finalproject') + "/scripts/coin.png", 1), toCoord, self.cv_image)
                    cur_image = self.warpAndCombine(cv2.imread(self.rospack.get_path('finalproject') + "/scripts/coin.png", 1), toCoord, cur_image)
        # cv2.imshow('video_window', self.cv_image)
        cv2.imshow('video_window', cur_image)
        cv2.waitKey(10)


    def centerToCorners(self,center):
        width = 0.2 
        return np.array([[[center[0]],[center[1]],[0],[1]],[[center[0]],[ceneter[1]],[width],[1]],[[center[0]],[center[1]+width],[0],[1]],[[center[0]],[center[1]+width],[width],[1]]])

        """
        def run(self):
        This function executes the program 
        """
    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(10)
        thetaIncrement = pi/15
        theta = pi/2

        centerOfCoin  = [1,0]
        self.coinInWorld.append(centerOfCoin)
        # center is not actually center it will be left bottom corner
        coin_coord = self.centerToCorners(centerOfCoin)
        # coin_coord = np.array([[[1], [0], [0], [1]],[[1], [0], [.2], [1]],[[1], [.2], [0], [1]],[[1], [.2], [.2], [1]]],dtype='float32')
        

        while not rospy.is_shutdown():
            if self.K != None and self.cv_image != None:
            # try:
                # takes the coordinate in the world and rotate it by theta
                # rotated is a coordinates in the world
                rotated = self.rotatePoints(coin_coord, theta)
                theta += thetaIncrement
                #this function does the transformation in between coordinate system, and objects' pixelated values
                #save it in an array of objects
                self.transformWorldToCamera(rotated)
                self.render_coin()
    #           r.sleep()
            # except Exception as inst:
            #     print inst

if __name__ == '__main__':
    nb = Neatobot()
    nb.run()