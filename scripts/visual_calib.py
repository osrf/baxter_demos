"""Use PCL data from the Kinect to figure out the pose needed to grasp the desired object.
Inputs:
/camera/depth/... (kinect frame)

How to calculate transform from camera PCL to robot space?
Visual self-calibration: Recognize the hand and estimate position of the camera based on hand position

Measure relative to head?
"""

import sys
import argparse

import rospy
import baxter_interface

import cv, cv2, cv_bridge
import numpy
import tf

import common
import ik_command

from sensor_msgs import (
    Image,
    CameraInfo
)

import image_geometry

from object_finder import ProcessSubscriber.colorSegmentation #TODO: Put in common or another library

#TODO: rosparam
marker_side = .035
marker_translate = [-.015, 0, -(marker_side/2+.07) ] #Assuming marker NOT on side with the camera

def flat_index(point, width):
    return point[1]*width + point[0]

class VisualCalibrator():
    def __init__(limb, cascade_filename):
        # Init Cascade Classifier
        #self.cascade = cv2.CascadeClassifier(cascade_filename)
        # if filename doesn't exist, train (do outside of this class?)

        #For now: use object_finder node to track the purple color of the markers, get a rotated rectangle around the contour

        self.limb = limb
        self.limb_if = baxter_interface.Limb(limb)
        self.tf = tf.TransformListener()


        
        cv2.createTrackbar("blur", "Processed image", 12, 50, nothing)
        cv2.createTrackbar("radius", "Processed image", 30, 128, self.updateRadius)
        cv2.createTrackbar("open", "Processed image", 4, 15, nothing)


        # Subscribe to /camera/depth_registered/points, /camera/rgb/image_color (or should I extract the 2D image from PC?), /camera/depth_registered/camera_info
    def depth_cam_callback(data):
        self.depth_msg = data
        #unregister callback

        if self.color_msg is not None:
            camera_model_init()

    def color_cam_callback(data):
        self.color_msg = data
        #unregister callback
        if self.depth_msg is not None:
            camera_model_init()
        

    def camera_model_init():
        self.camera_model = image_geometry.StereoCameraModel(self.color_msg , self.depth_msg)
        

    def cascadeDetect(self, img):
        # gray the image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Use pre-trained cascade classifier to detect marker on
        # Not sure about these params...
        ROIs = self.cascade.detectMultiScale(gray, 1.3, 5) 

        # If there are multiple ROIs, go with the biggest?
        # Since these are OpenCV rectangles, the last two coordinates are the width and height
        ROI = max(ROIs, key = lambda x : x[2]*x[3])
        #Do a little more preprocessing so that we can find a pose
        subimg = gray[ROI[1]:ROI[1]:ROI[3], ROI[0]:ROI[0]+ROI[2]]
        canny = cv2.Canny(subimg, 0, 80)
        contours = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def colorDetect(img):
        point = self.clickpoint
        blur_radius = cv2.getTrackbarPos("blur", "Processed image")
        radius = cv2.getTrackbarPos("radius", "Processed image")
        open_radius = cv2.getTrackbarPos("open", "Processed image")

        self.processed = ProcessSubscriber.colorSegmentation()

        contours, hierarchy = cv2.findContours(self.processed, cv2.CV_RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #Sort the contours by area and get the two biggest
        contourpts = numpy.concatenate()
        
        rect_rot = cv2.minAreaRect(contourpts)
        centroid = 
        #Calculate the position
        

    def findGripper(img_2d, pc):
        # Input: 2D image from Kinect and point depth
        
        pos, orientation = colorDetect(img_2d)

        #TODO: check indexing
        point = pc.PointField[ flat_index(pos, pc.width) ]

        # Convert scale to robot units. OpenNI topics provide camera_info
        # Hmm. Maybe should use /camera/depth_registered/disparity instead? No indexing needed
        marker_camera = self.camera_model.projectPixelTo3d(point[0:2], point[3] )

        # Transform marker position to base frame
        trans, rot = self.tf.lookupTransform('/'+limb+'_gripper', '/base', rospy.Time(0) )
        gripper_to_base = tf.fromTranslationRotation(trans, rot)
        marker_gripper = numpy.array(marker_translate+[1]).reshape(4, 1)       
        #Take off the silly homogeneous bit
        marker_base = numpy.dot(gripper_to_base, marker_gripper)[0:3]

        # Now that we have the pose of the marker in two different frames,
        # we can solve for the transform between these frames (camera and robot base frame)
        

def main():
    #Move arms to preset position where markers are visible
    cv2.namedWindow("Processed image")

if __name__ == "__main__":
    main()
