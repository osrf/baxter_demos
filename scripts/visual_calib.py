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
import scipy
import tf
import image_geometry
import camera_calibration_parser

from math import cos, sin

import common
import ik_command

from sensor_msgs.msg import (
    Image,
    CameraInfo,
    PointCloud2
)

from geometry_msgs.msg import Pose
from stereo_msgs.msg import DisparityImage
from visualization_msgs.msg import Marker
import visualization_msgs.msg



#TODO: rosparam
marker_side = .035
marker_height = 0.06735
marker_x = -0.03829
marker_gripper_points = [ [marker_x, -marker_side/2, marker_side+marker_height], [marker_x, marker_side/2, marker_side+marker_height], [marker_x, marker_side/2, marker_height], [marker_x, -marker_side/2, marker_height] ]
#marker_gripper_points = [ [marker_x, 0, marker_side + marker_height], [marker_x, 0, marker_height] ]

#translation from the center of the marker to the gripper
marker_translate = [-marker_x, 0, -marker_side/2-marker_height]

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
        self.ir_msg = None
        self.color_msg = None
        self.camera_model = image_geometry.StereoCameraModel()
        self.camera_model_init()
        self.color = None

        self.point_sub = rospy.Subscriber("/camera/depth_registered/disparity", DisparityImage, self.depth_assign)
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.color_assign)
        
    def publish(self, rate=100):
        self.handler_pub = rospy.Publisher("visual_calib/"+self.limb+"/camera_pose", Pose)
        self.pub_rate = rospy.Rate(rate)
        self.msg = None

    def setClickpoint(self, x, y):
        self.clickpoint = (x, y)

    def depth_assign(self, data):
        self.disparity_img = cv_bridge.CvBridge().imgmsg_to_cv2( data.image )

    def camera_model_init(self):
        print "Initializing camera model"
        path = rospy.get_param("/visual_calib/yaml_path")
        # Parse cameraInfo objects from yaml files
        color_cameraInfo = camera_calibration_parser.parse_yaml(path+"rgb_calib.yaml")
        print color_cameraInfo
        ir_cameraInfo = camera_calibration_parser.parse_yaml(path+"ir_calib.yaml")
        print ir_cameraInfo
        self.camera_model.fromCameraInfo(color_cameraInfo, ir_cameraInfo)

        
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
            return None
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
            return None
        contourpts = contours[0]
      
        rot = cv2.minAreaRect(contourpts)
        p1 = numpy.array(rot[0])
        s1, s2 = rot[1]
        theta = -abs(rot[2])
        #x1, y1, s1, s2, theta = rot
        #where x1, y1 is the top left point, s1, s2 are the sides, and theta is the rotation
        #TODO: check the negative sign
        
        #Calculate the four points of the rectangle
        rect = cv2.cv.BoxPoints(rot)
        return rect
        
        #points = [ p1 + numpy.array((-s1/2*cos(theta), -s1/2*sin(theta) )) ]
        #points.append( p1 + numpy.array((s1/2*cos(theta), s1/2*sin(theta))) )
        
        #return points, p1

    def findGripper(self, img_2d, pc):
        # Input: 2D image from Kinect and point depth
        # Returns transform from camera frame to robot frame
        self.processed = numpy.zeros(self.processed.shape)
        
        points = self.colorDetect(img_2d)
        if points is None:
            return

        cv2.line(self.processed, tuple(map(int, points[0])), tuple(map(int, points[1])), 255, 2)
        cv2.line(self.processed, tuple(map(int, points[1])), tuple(map(int, points[2])), 255, 3)
        cv2.line(self.processed, tuple(map(int, points[2])), tuple(map(int, points[3])), 255, 3)
        cv2.line(self.processed, tuple(map(int, points[3])), tuple(map(int, points[0])), 255, 3)
       
        #Get some 3D
        if self.disparity_img is None:
            print "Waiting on disparity"
            return

        marker_camera = []

        #Miiight need to translate pixels so that 0, 0 is the center of the frame
        for point in points:
            disparity = self.disparity_img[point[1], point[0]] 
            print "got disparity:", disparity
            #point = (point[0]-320, point[1]-200)
            # Convert scale to robot units. OpenNI topics provide camera_info
            p_marker = numpy.array(self.camera_model.projectPixelTo3d(point, disparity))
            p_marker = p_marker.reshape((3,1))
            marker_camera.append(p_marker)

        print "extracted points:", marker_camera
        distances = [numpy.linalg.norm(marker_camera[i]-marker_camera[i-1]) for i in range(len(marker_camera))]
        print "Distances between extracted points:", distances
 
        # Transform marker position to base frame
        self.tf.waitForTransform('/base', '/'+self.limb+'_gripper', rospy.Time(), rospy.Duration(4.0))
        trans, rot = self.tf.lookupTransform('/base', '/'+self.limb+'_gripper', rospy.Time(0) )

        gripper_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
        #R_gripper_base = gripper_to_base[0:3, 0:3]
        #t_gripper_base = gripper_to_base[0:3, 3]

        marker_base = []
        for marker_gripper in marker_gripper_points:
            marker_gripper = numpy.array(marker_gripper+[1]).reshape(4, 1)       

            marker_base.append( numpy.dot(gripper_to_base, marker_gripper)[0:3] )

        distances = [numpy.linalg.norm(marker_base[i]-marker_base[i-1]) for i in range(len(marker_base))]
        print "Distances between points in robot frame:", distances
        # Now that we have the pose of the marker in two different frames,
        # we can solve for the transform between these frames (camera and robot base frame)
        # Solution from here:
        # http://math.stackexchange.com/questions/222113/given-3-points-of-a-rigid-body-in-space-how-do-i-find-the-corresponding-orienta 
        #TODO: check correspondences between point sets

        #this doesn't work because the sides are not quite the same size, also correspondences might not be great
        """P = numpy.hstack( (marker_camera[1]-marker_camera[0], marker_camera[2]-marker_camera[0], marker_camera[3]-marker_camera[0] ) )
        Q = numpy.hstack( (marker_base[1]-marker_base[0], marker_base[2]-marker_base[0], marker_base[3]-marker_base[0] ) )
        try:
            R = numpy.dot(Q, numpy.linalg.inv(P))
            T = marker_base[0] - numpy.dot(Q, numpy.dot(numpy.linalg.inv(P), marker_camera[0]) )
        except numpy.linalg.linalg.LinAlgError:
            print "Singular matrix"
            return
        print "Rotation:", R
        print "Translation:", T"""
        P = numpy.hstack( tuple(marker_camera)).transpose()
        Q = numpy.hstack( tuple(marker_base)).transpose()
        retval, out, outliers = cv2.estimateAffine3D(P, Q)
        print "Affine transformation:", out
        A = out[0:3, 0:3]
        T = out[:, 3]
        R = A/numpy.linalg.norm(A[0, :])
        print "'Rotation:'", R
        print "Translation:", T

        #S = tf.transformations.scale_from_matrix(A)
        #print "Scale factor:", S
        S = numpy.zeros((2, 3))
        for i in range(3):
            S[0, i] = numpy.linalg.norm(A[i, :])
            S[1, i] = numpy.linalg.norm(A[:, i])
        print "Scale factors:", S
            

        #Turn rotation matrix into quaternion
        try:
            quat = tf.transformations.quaternion_from_euler(tf.transformations.rotation_from_matrix(R))
        except:
            print "Rotation matrix had eigenvalues:", numpy.linalg.eig(R)[1]
            
            return
        position = numpy.dot(numpy.linalg.inv(R), T)
        camera_pose = Pose( position = position, orientation = quat )
        #Publish as a marker to display in rviz
        self.msg = Marker(type=visualization_msgs.msg.Marker.ARROW, pose = camera_pose, scale = (1, 1, 1))
        

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
    while not ml.done and not rospy.is_shutdown():
        if calib.img_2d is not None:
            cv2.imshow("Camera", calib.img_2d)

        cv2.waitKey(100)
    calib.setClickpoint(ml.x_clicked, ml.y_clicked)
    calib.publish()

    while not rospy.is_shutdown():
        calib.main_callback()
        cv2.imshow("Processed image", calib.processed)
        cv2.imshow("Camera", calib.img_2d)
        cv2.waitKey(100)
        if calib.msg is not None:
            calib.handler_pub.publish(calib.msg)
            rospy.sleep(calib.pub_rate)

if __name__ == "__main__":
    main()
