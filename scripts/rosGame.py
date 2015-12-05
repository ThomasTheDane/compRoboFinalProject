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

	def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('neato_bot')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.D= 0
        self.K= 0
        self.cx = 0
        self.cy = 0
        self.cz = 0
        self.theta = 0

		rospy.Subscriber("/STAR_pose_euler_angle", Vector3, self.processEulerAngle)	
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        rospy.Subscriber("STAR_pose_continuous",PoseStamped, self.processLocation)
        rospy.Subscriber("/camera/camera_info", sensor_msgs, self.get_camerainfo)
        # self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('ARwindow')

    def processLocation(self,msg):
    	self.cx = msg.pose.position.x
    	self.cy = msg.pose.position.y
    	self.cz = msg.pose.position.z

    def processEulerAngle(self, vec3):
    	self.angle = vec3[2]

    def transformWorldToCamera(self, coord):
		#/STAR_pose_euler_angle
		rotat = rotation_matrix(self.theta,[0,0,1])
		trans = np.matrix([[1,0,0,self.cx],[0,1,0,self.cy],[0,0,1,self.cz],[0,0,0,1]])

		new_coord = np.dot(rotat, np.dot(trans, coord))
		pixelate(new_coord)

	def pixelate(self, new_coord):
		points = cv2.projectPoints(new_coord, (0,0,0), (0,0,0), self.K, self.D)
		return points

    def get_camerainfo(self,msg):
    	self.D = msg.D
    	self.K = msg.K

    def process_image(self,msg):
    	self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow('ARwindow', self.cv_image)
        cv2.waitKey(5)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
        	# start out not issuing any motor commands
            r.sleep()

if __name__ == '__main__':
    nb = Neatobot()
    nb.run()
