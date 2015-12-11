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
        # rospy.init_node('AR')                       # the latest image from the camera
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

        # rospy.Subscriber("STAR_pose_euler_angle", Vector3, self.processEulerAngle)  
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

    def transformWorldToCamera(self, coord):
        rotat = rotation_matrix(self.theta,[0,0,1])
        trans = np.matrix([[1,0,0,-self.cx],[0,1,0,-self.cy],[0,0,1,-self.cz],[0,0,0,1]])
        new_coord = np.dot(rotat, np.dot(trans, coord))
        # print new_coord[:3]
        self.pixelate(new_coord[:3])

    def pixelate(self, new_coord):

        self.new_coord = [-new_coord[1,0], -new_coord[2,0], new_coord[0,0]]
        print "new_coord after swap",new_coord
        new_coord = np.array([new_coord])
        
        # points = cv2.projectPoints(new_coord, (0,0,0), (0,0,0), self.K, self.D)
        # self.points = points[0]
        # print self.K
        p = self.K.dot(new_coord.T)
        self.pixel =  p / p[2]
        print "self.pixel", self.pixel
        

    def get_camerainfo(self,msg):
        self.D = msg.D
        self.K = np.resize(msg.K, (3,3))

    def process_image(self,msg):

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.pixel !=None :
            # print self.pixel[0], self.pixel[1]
            # print  "z coordinate", self.new_coord[2]
            # if self.new_coord[2] > 0:
            cv2.circle(self.cv_image, (int(self.pixel[0]), int(self.pixel[1])), 100, (0, 0, 255))
        
        cv2.imshow('video_window', self.cv_image)
        cv2.waitKey(10)

    def 

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.theta!=0:
                coin_coord = np.array([[1], [0], [0], [1]], dtype='float32')
                # coin_coord = np.array([coin_coord])
                self.transformWorldToCamera(coin_coord)
                # self.transformWorldToCamera([[2],[1],[1],[1]])
            r.sleep()

if __name__ == '__main__':
    nb = Neatobot()
    nb.run()
