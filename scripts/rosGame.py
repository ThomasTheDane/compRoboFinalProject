#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
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
import random

from ARSprite import ARSprite

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
        self.coinsInWorld = []
        self.spikesInWorld = []
        self.score=0

        rospy.Subscriber("STAR_pose_continuous",PoseStamped, self.processLocation)
        rospy.Subscriber("camera/camera_info", CameraInfo, self.get_camerainfo)
        self.score_pub = rospy.Publisher("score", Int32, queue_size=1)
        """
        def processLocation(self,msg):
        A callBack function for STAR_pose_continuous which saves the location of the robot
        in the world coordinate system, calculates and save the angle of robot's Header
        """        

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
        self.theta = -euler_angles[2]
        self.checkStatus(self.cx,self.cy)

    def checkScore(self, x,y):
        padding = 0.2
        for i, coin in enumerate(self.coinsInWorld):
            if abs(x-coin.midline[0]) < padding and abs(y-coin.midline[1]) < padding:
                self.score+=1
                self.coinsInWorld.pop(i)
                self.score_pub.publish(Int32(self.score))

    def checkSpike(self, x,y):
        padding = 0.3
        for i, spike in enumerate(self.spikesInWorld):
            if abs(x-spike.midline[0]) < padding and abs(y-spike.midline[1]) < padding:
                #TODO Send word to web that you've died 
                self.startGame()

    def checkStatus(self,x,y):
        self.checkScore(x,y)
        self.checkSpike(x,y)

        """
        def transformWorldToCamera(self, aCoin):
        A function that transforms the coordinate systems, from world coordinate system to 
        a camera coordinate system.
        """
  
    def get_camerainfo(self,msg):
        self.D = msg.D
        self.K = np.resize(msg.K, (3,3))

        """
        def process_image(self,msg):        
        """
    def process_image(self,msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def centerToCorners(self,center):
        width = 0.2 
        return np.array([[[center[0]],[center[1]],[0],[1]],[[center[0]],[center[1]],[width],[1]],[[center[0]],[center[1]+width],[0],[1]],[[center[0]],[center[1]+width],[width],[1]]],dtype='float32')

    def distanceBetween(self, pointA, pointB):
        return abs(math.sqrt(((pointB[0] - pointA[0])**2) + ((pointB[1] - pointA[1])**2)))

    def startGame(self):
        self.score = 0

        # place 5 random coins in 3x3 space 
        for i in range(0, 5):
            print "making coin"
            coin_image_path = "/scripts/coin.png"
            coin_speed = pi/60
            coin_coordinate = [random.uniform(0,3), random.uniform(-3,0)]
            arCoinSprite = ARSprite(self.centerToCorners(coin_coordinate), coin_image_path, coin_speed)
            self.coinsInWorld.append(arCoinSprite)

        # place 3 randomly placed spikes 
        for i in range(0,3):
            spike_image_path = "/scripts/spike.jpg"
            spike_coordinate = [random.uniform(0,3), random.uniform(-3,0)]
            valid_coordinate = False
            while not valid_coordinate:
                for aCoin in self.coinsInWorld:
                    if self.distanceBetween(aCoin.midline, spike_coordinate) < .2:
                        valid_coordinate = False
                        spike_coordinate = [random.uniform(0,3), random.uniform(0,3)]
                        break
                valid_coordinate = True
            arSpikeSprite = ARSprite(self.centerToCorners(spike_coordinate), spike_image_path)
            self.spikesInWorld.append(arSpikeSprite)

        """
        def run(self):
        This function executes the program 
        """
    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(10)

        self.startGame()

        while not rospy.is_shutdown():
            if self.cv_image != None and self.K != None:
                newImage = self.cv_image
                
                # construct list of tuples: (distanceToRobot, spriteObject)
                distanceRenderArray = []
                for aCoin in self.coinsInWorld:
                    distanceRenderArray.append((self.distanceBetween([self.cx, self.cy], aCoin.midline), aCoin))

                for aSpike in self.spikesInWorld:
                    distanceRenderArray.append((self.distanceBetween([self.cx, self.cy], aSpike.midline), aSpike))
                
                distanceRenderArray.sort(key=lambda tup: tup[0], reverse=True)

                for _, aSprite in distanceRenderArray:
                    aSprite.setK(self.K)
                    newImage = aSprite.addSpriteToView(newImage, [self.cx, self.cy, self.cz], self.theta)

                cv2.imshow('video_window', newImage)
                cv2.waitKey(10)

                
if __name__ == '__main__':
    nb = Neatobot()
    nb.run()