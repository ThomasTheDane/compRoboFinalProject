from copy import deepcopy
from math import sin, cos, pi, atan2, fabs
import rospkg
import math
import cv2
import numpy as np
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3
import rospkg


class ARSprite:
    def __init__(self, fourConers, imagePath, rotatationSpeed=0):
        self.fourConers = fourConers
        self.rospack = rospkg.RosPack()
        self.image = cv2.imread(self.rospack.get_path('finalproject') + imagePath, 1)
        self.K = None
        self.rotatationSpeed = rotatationSpeed
        self.spriteVisible = True
        self.spriteTheta = 0
        self.midline = [fourConers[0][0][0] + ((fourConers[1][0][0] - fourConers[0][0][0]) / 2.0), fourConers[0][1][0] + ((fourConers[1][1][0] - fourConers[0][1][0]) / 2.0)]

    def setK(self, K):
        self.K = K

    def transformWorldToPixels(self, aSpriteCordinates, cameraLocation, cameraAngle):
        #initialize the array which has elements of coins
        #initialize the camera coordinate points for each coin image
        aSpritePixels = []

        for aCoord in aSpriteCordinates:
            #rotational matrix, rotate by self.theta, along the z-axis
            rotat = rotation_matrix(cameraAngle,[0,0,1])
            #translation matrix which maps the location of a robot(in world) to camera coordinate origin
            trans = np.matrix([[1,0,0,-(cameraLocation[0])],[0,1,0,-(cameraLocation[1])],[0,0,1,-(cameraLocation[2])],[0,0,0,1]])
            #this calculates the points of the aCoin in camera coordinate system
            new_coord = np.dot(rotat, np.dot(trans, aCoord))
            #calls the pixelate function which maps the camera coordinate points into 2D screen
            aSpritePixels.append(self.pixelate(new_coord[:3]))
        #append to the array which takes an array for each coin
        return aSpritePixels


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
            self.spriteVisible = False
        else:
            self.spriteVisible = True
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
        imageWidth = 0.2
        #based on how we defined the points in the world coordinate for an image, 
        #this rotation will make the two side corner point pair rotate the a same amount 
        points[0][0] = self.midline[0] + (math.cos(theta)*(imageWidth*math.sqrt(2)/2))
        points[0][1] = self.midline[1] + (math.sin(theta)*(imageWidth*math.sqrt(2)/2))
        points[1][0] = self.midline[0] + (math.cos(theta)*(imageWidth*math.sqrt(2)/2))
        points[1][1] = self.midline[1] + (math.sin(theta)*(imageWidth*math.sqrt(2)/2))
        points[2][0] = self.midline[0] + (math.cos(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
        points[2][1] = self.midline[1] + (math.sin(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
        points[3][0] = self.midline[0] + (math.cos(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
        points[3][1] = self.midline[1] + (math.sin(theta+math.pi)*(imageWidth*math.sqrt(2)/2))
            
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
#        print "newWidth is ", newWidth, "newHeight is ", newHeight

        finalImage = destinationImage

        if newWidth < 0 or newHeight < 0 or newWidth > 5000 or newHeight > 5000:
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
        #cv2.imshow('roi', roi)
        #cv2.waitKey(5)

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

        """
        def render_coin(self):
        
        
        """
    def renderSprite(self, spritePixels, destinationImage):
        toCoor = []

        if spritePixels:
            #print "spritePixels", spritePixels
            if self.spriteVisible:
                for aPixel in spritePixels:
                    cv2.circle(destinationImage,(int(aPixel[0]), int(aPixel[1])), 5, (0, 0, 255))
                    toCoor.append([int(aPixel[0]), int(aPixel[1])])
                #print type(spritePixels)
                destinationImage = self.warpAndCombine(self.image, toCoor, destinationImage)

        return destinationImage

    def addSpriteToView(self, destinationImage, cameraLocation, cameraAngle):
        if self.K != None:
            if(self.rotatationSpeed != 0):
                self.fourConers = self.rotatePoints(self.fourConers, self.spriteTheta)
                self.spriteTheta += self.rotatationSpeed

            spritePixels = self.transformWorldToPixels(self.fourConers, cameraLocation, cameraAngle)
            #print "sprite pixels", spritePixels
            return self.renderSprite(spritePixels, destinationImage)
        else:
            print "NO K"
            return destinationImage