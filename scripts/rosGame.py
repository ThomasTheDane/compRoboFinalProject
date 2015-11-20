#!/usr/bin/env python

""" This code implements a ceiling-marker based localization system.
	The code is slightly modified based on the number/kinds of markers 
	we've decided to use.
    The core of the code is filling out the marker_locators
    which allow for the specification of the position and orientation
    of the markers on the ceiling of the room """

import rospy
from neatoLocation import MarkerProcessor

class Neatobot:
	"""One player/robot"""
	
if __name__ == '__main__':
    nh = MarkerProcessor(use_dummy_transform=True)
    nh.run()
