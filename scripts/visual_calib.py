#!/usr/bin/env python
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

from math import cos, sin

import common
import ik_command

from sensor_msgs.msg import (
    Image,
    CameraInfo,
    PointCloud2
)

from stereo_msgs.msg import DisparityImage

import image_geometry


#TODO: rosparam
marker_side = .035
marker_bottom = [-.015, 0, -(marker_side+.07) ]
marker_top = [-.015, 0, -.07 ]
marker_translate = [-.015, 0, -(marker_side/2+.07) ] #Assuming marker NOT on side with the camera

def nothing(data):
    pass

def cleanup():
    cv2.destroyAllWindows()


class VisualCalibrator():
    def __init__(self, limb, cascade_filename=None):
        # Init Cascade Classifier
        #self.cascade = cv2.CascadeClassifier(cascade_filename)
        # if filename doesn't exist, train (do outside of this class?)

        #For now: use object_finder node to track the color of the markers, get a rotated rectangle around the contour

        self.limb = limb
        self.limb_if = baxter_interface.Limb(limb)
        self.tf = tf.TransformListener()
        
        cv2.createTrackbar("blur", "Processed image", 12, 50, nothing)
        cv2.createTrackbar("radius", "Processed image", 30, 128, nothing)
        cv2.createTrackbar("open", "Processed image", 4, 15, nothing)

        self.img_2d = numpy.zeros((640, 400)) 
        self.disparity_img = None
        self.processed = numpy.zeros((640, 400))
        self.contour_img = numpy.zeros((640, 400))
        self.clickpoint = None
        self.pointcloud = None
        self.depth_msg = None
        self.color_msg = None
        self.camera_model = image_geometry.StereoCameraModel()
        self.color = None

        self.point_sub = rospy.Subscriber("/camera/depth_registered/disparity", DisparityImage, self.depth_assign)
        self.img_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.color_assign)
        self.depth_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_cam_callback)
        self.color_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.color_cam_callback)

    def setClickpoint(self, x, y):
        self.clickpoint = (x, y)

    def depth_assign(self, data):
        self.disparity_img = cv_bridge.CvBridge().imgmsg_to_cv2( data.image )

    def depth_cam_callback(self, data):
        print "Got depth camera info"
        self.depth_msg = data

        if self.color_msg is not None:
            self.camera_model_init()

    def color_cam_callback(self, data):
    
        print "Got color camera info"
        self.color_msg = data

        if self.depth_msg is not None:
            self.camera_model_init()
        

    def camera_model_init(self):
        print "Initializing camera model"
        print self.color_msg.D
        print self.depth_msg.D
        self.camera_model.fromCameraInfo(self.color_msg, self.depth_msg)

        self.depth_info_sub.unregister()
        self.color_info_sub.unregister()
        
    def color_assign(self, data):
        self.img_2d = cv_bridge.CvBridge().imgmsg_to_cv2(data)

    def main_callback(self):
        if self.img_2d is not None:
            self.findGripper(self.img_2d, self.pointcloud)

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

    def colorDetect(self, img):
        #If I replace the markers with big purple squares this will work slightly better
        #Need to solve the shift axis problem
        point = self.clickpoint
        if point is None:
            return None, None
        blur_radius = cv2.getTrackbarPos("blur", "Processed image")
        radius = cv2.getTrackbarPos("radius", "Processed image")
        open_radius = cv2.getTrackbarPos("open", "Processed image")

        blur_radius = blur_radius*2-1

        blur_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(numpy.uint8)
        if blur_radius > 0:
            blur_img = cv2.GaussianBlur(img, (blur_radius, blur_radius), 0)
        else:
            blur_img = img

        if self.color == None:
            self.color = blur_img[point[1], point[0]]

        self.processed = common.colorSegmentation(blur_img, blur_radius, radius, open_radius, self.color)

        contours, hierarchy = cv2.findContours(self.processed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #self.contour_img = numpy.zeros(self.img_2d.shape)
        cv2.drawContours(self.processed, contours, -1, 255)

        #Sort the contours by area and get the two biggest
        contours.sort(key = lambda x : cv2.contourArea(x), reverse = True)
        if len(contours) == 0:
            return None, None
        contourpts = contours[0]
        """if len(contours) > 1:
            contourpts = numpy.concatenate((contours[0], contours[1]))
        elif len(contours) > 0:
            contourpts = contours[0]
        else:
            print "Couldn't find marker contours"
            return None, None"""
        
        rot = cv2.minAreaRect(contourpts)
        x1, y1 = rot[0]
        s1, s2 = rot[1]
        theta = abs(rot[2])
        #x1, y1, s1, s2, theta = rot
        #where x1, y1 is the top left point, s1, s2 are the sides, and theta is the rotation
        #TODO: check the negative sign
        
        #Return the vertical axis through the centroid
        p1 = (x1+s1/2*cos(theta), y1-s1/2*sin(theta))
        p2 = (x1-s2*sin(theta)+s1/2*sin(theta), y1-s2*cos(theta)+s1/2*cos(theta))

        return p1, p2

    def findGripper(self, img_2d, pc):
        # Input: 2D image from Kinect and point depth
        # Returns transform from camera frame to robot frame
        
        p1_2d, p2_2d = self.colorDetect(img_2d)
        if p1_2d is None or p2_2d is None:
            return
        cv2.line(self.processed, tuple(map(int, p1_2d)), tuple(map(int, p2_2d)), 255, 3)
        
        #Get some 3D
        if self.disparity_img is None:
            print "Waiting on disparity"
            return
        p_2d = [p1_2d, p2_2d]

        marker_camera = []
        #Miiight need to translate pixels so that 0, 0 is the center of the frame
        for i in range(2):
            disparity = self.disparity_img[p_2d[i][1], p_2d[i][0]] 
            print "Got disparity:", disparity

            # Convert scale to robot units. OpenNI topics provide camera_info
            p_marker = numpy.array(self.camera_model.projectPixelTo3d(p_2d[i], disparity))
            p_marker = numpy.concatenate((p_marker, numpy.ones(1))).reshape((4,1))
            marker_camera.append(p_marker)
            print "Got marker in camera frame:", p_marker
        # Transform marker position to base frame
        self.tf.waitForTransform('/base', '/'+self.limb+'_gripper', rospy.Duration(4.0))
        trans, rot = self.tf.lookupTransform('/base', '/'+self.limb+'_gripper', rospy.Time(), rospy.Duration(4) )

        gripper_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
        marker_top_g = numpy.array(marker_top+[1]).reshape(4, 1)       
        marker_bottom_g = numpy.array(marker_bottom+[1]).reshape(4, 1)

        marker_top_base = numpy.dot(gripper_to_base, marker_top_g)
        marker_bottom_base = numpy.dot(gripper_to_base, marker_bottom_g)

        # Now that we have the pose of the marker in two different frames,
        # we can solve for the transform between these frames (camera and robot base frame)
        # Solve a formulation of Wahba's problem using SVD
        B = numpy.zeros( (4, 4) )
        B += numpy.dot(marker_top_base, marker_camera[0].transpose() )
        B += numpy.dot(marker_bottom_base, marker_camera[1].transpose() )
        U, S, V = numpy.linalg.svd(B)
        M = numpy.identity(4)
        M[2, 2] = numpy.linalg.det(U)*numpy.linalg.det(V)
        T = U*M*V
        
        #This should work...?
        print T
        
        

def main():
    #TODO: Move arms to preset position where markers are visible

    cv2.namedWindow("Processed image")
    cv2.namedWindow("Camera")

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    
    parser.add_argument('-m', '--method', choices=['color', 'edge', 'star', 'watershed'],
        required=False, help='which object detection method to use')

    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    if args.method is None:
        args.method = 'watershed'

    print("Initializing node... ")
    rospy.init_node("visual_calib_%s" % (limb,))
    rospy.on_shutdown(cleanup)

    calib = VisualCalibrator(limb)

    print("Click on the object you would like to track, then press any key to continue.")
    ml = common.MouseListener()

    cv2.setMouseCallback("Camera", ml.onMouse)
    while not ml.done:
        if calib.img_2d is not None:
            cv2.imshow("Camera", calib.img_2d)

        cv2.waitKey(100)
    calib.setClickpoint(ml.x_clicked, ml.y_clicked)


    while not rospy.is_shutdown():
        calib.main_callback()
        cv2.imshow("Processed image", calib.processed)
        cv2.imshow("Camera", calib.img_2d)
        cv2.waitKey(100)

if __name__ == "__main__":
    main()
