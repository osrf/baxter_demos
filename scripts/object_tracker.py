#!/usr/bin/env python

"""Simple Object Tracker by Jackie Kay (jackie@osrfoundation.org)
Select one of several object recognition methods and an object in Baxter's hand camera to track.
Baxter will try to keep the camera aligned with the object centroid.
"""

import sys
import argparse
from math import sqrt, floor
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import common
import ik_command
import cv2, cv_bridge
import numpy
import tf
from sensor_msgs.msg import (
    Image,
)

alpha = 0.001

class CameraSubscriber:
    def subscribe(self, limb):
        self.limb = limb
        self.handler = rospy.Subscriber("/cameras/"+limb+"_hand_camera/image", Image, self.callback)

    def unsubscribe(self):
        self.handler.unregister()

    def callback(self, data):
        self.get_data(data)
        cv2.imshow("Hand camera", self.img)
        cv2.waitKey(100)

    def get_data(self, data):
        img = cv_bridge.CvBridge().imgmsg_to_cv2(data)
        self.img = img

class ProcessSubscriber(CameraSubscriber):
    def __init__(self, method, point):

        #TODO: write wrappers for edge and color detection
        if method == 'edge':
            cv2.createTrackbar("threshold 1", "Processed image", 500, 2000, nothing)
            cv2.createTrackbar("threshold 2", "Processed image", 10, 2000, nothing)
            self.detectFunction = self.edgeDetect
        elif method == 'color':
        #TODO: make color detection work, ever.
            cv2.createTrackbar("blur", "Processed image", 2, 15, nothing)
            cv2.createTrackbar("radius", "Processed image", 10, 128, nothing)
            cv2.createTrackbar("open", "Processed image", 3, 15, nothing)
            cv2.createTrackbar("close", "Processed image", 3, 15, nothing)
            self.detectFunction = self.colorDetect

        elif method == 'star':
            #TODO: sliders
            maxSize = 45 # Maximum number of filters to apply?
            responseThreshold = 40 # higher = fewer features retrieved
            lineThresholdProjected = 15  # maximum ratio between Harris of responses. higher = eliminates more edges
            lineThresholdBinarized = 20 #maximum ratio between Harris of sizes. higher = more points
            cv2.createTrackbar("Response threshold", "Processed image", 20, 90, self.updateDetector)
            cv2.createTrackbar("Projected line threshold", "Processed image", 3, 30, self.updateDetector)
            cv2.createTrackbar("Binarized line threshold", "Processed image", 3, 30, self.updateDetector)
            self.detector = cv2.StarDetector(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized) 
            self.detectFunction = self.starDetect

        self.point = point
        self.centroid = point
    def updateDetector(self):
        maxSize = 45
        responseThreshold = cv2.getTrackbarPos("Response threshold", "Processed image")
        lineThresholdProjected = cv2.getTrackbarPos("Projected line threshold", "Processed image")
        lineThresholdBinarized = cv2.getTrackbarPos("Binarized line threshold", "Processed image")
        self.detector = cv2.StarDetector(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized) 

    def callback(self, data):
        CameraSubscriber.get_data(self, data)
        #Process image
        cv2.imshow("Hand camera", self.img)
        self.processed = self.detectFunction(self.img, self.point)
        cv2.imshow("Processed image", self.processed)
        
        #Find the contour associated with self.point
        contour_img = self.processed.copy()
        contours, hierarchy = cv2.findContours(contour_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None or len(contours) == 0:
            cv2.waitKey(100)
            return
            
        cv2.drawContours(contour_img, contours, -1, (255, 255, 255))

        #Need a more sophisticated test, this does not work all the time
        for contour in contours:
            if cv2.pointPolygonTest(contour, self.point, False) > 0:
                #print "Found contour encircling desired point"
                break

        #Find the centroid of this contour
        self.centroid = (int(numpy.mean(contour[:, :, 0])), int(numpy.mean(contour[:, :, 1])) )
        #print "Found centroid:", self.centroid
        cv2.circle(img=contour_img, center=self.centroid, radius=3, color=(255, 255, 255), thickness=-1)
        cv2.imshow("Contours", contour_img)
        cv2.waitKey(100)

    def starDetect(self, img, point=None):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        keypoints = self.detector.detect(gray)
        #Render the key points
        n = len(keypoints)
        if n == 0:
            return

        # Now what to do with the key points? How to get thresholded image?
        # Put the keypoints in a np.f32 matrix, one column = one point
        samples = numpy.array( [point.pt for point in keypoints] )
        samples = numpy.float32(samples)

        # Cluster by size, determining the number of clusters by their compactness
        # TODO: input k?
        delta = 1000
        prev_compactness = sys.maxint
        for k in range(1, max(2, int(n/3))):
            max_iter = 10
            epsilon = 1.0
            attempts = 10
            compactness, labels, centers = cv2.kmeans(samples, k, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iter, epsilon), attempts, cv2.KMEANS_RANDOM_CENTERS)
            if prev_compactness - compactness < delta:
                if k > 1:
                    k-=1
                break
            prev_compactness = compactness
        # Put the output of k-means into a convenient dictionary
        print k
        labels = labels.flatten()
        polys = {} #Dictionary to group the polygons
        for i in range(labels.shape[0]):
            polys.setdefault(labels[i], []).append(samples[i, :])
        gray[:] = 0 
        # Draw polygon around clusters
        #TODO: load these values from file because calculating this is godawful
        phi = (1.0+sqrt(5))/2.0
        hues = [ numpy.array([floor( (i*phi - floor(i*phi)) * 179), 255, 255]) for i in range(0, 100, 10)]
        
        colors = [tuple(cv2.cvtColor(hue.reshape((1, 1, 3)).astype(numpy.uint8), cv2.COLOR_HSV2BGR).flatten().tolist()) for hue in hues]
        for i in polys.keys():
            points = numpy.array(polys[i]).astype(int)

            for point in points:
                # Draw clustered points in the same color
                cv2.circle(img=img, center=tuple(point.flatten().tolist()), radius=2, color=colors[i % len(colors)], thickness=-1)

            hull = cv2.convexHull(points)
            hull = hull.reshape((hull.shape[0], 1, 2))
            cv2.fillConvexPoly(gray, hull, color=255)
        cv2.imshow("Hand camera", img)
        return gray

    def edgeDetect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh1 = cv2.getTrackbarPos('threshold 1', "Processed image")
        thresh2 = cv2.getTrackbarPos('threshold 2', "Processed image")
        canny = cv2.Canny(gray, thresh1, thresh2)
        return canny

    def colorDetect(self, img):
        #Blur the image to get rid of those annoying speckles
        blur_radius = cv2.getTrackbarPos("blur", "Processed image")
        blur_radius = blur_radius*2-1
        blur_img = cv2.GaussianBlur(img, (blur_radius, blur_radius), 0)
        #Get color of point in image
        blur_img = img
        color = blur_img[self.point[0], self.point[1]]
        print color
        radius = cv2.getTrackbarPos("radius", "Processed image")
        #Grab the R, G, B channels as separate matrices
        #use cv2.threshold on each of them
        #AND the three images together
        bw = numpy.ones(img.shape[0:2], numpy.uint8)
        for i in range(3):
            if radius > color[i]:
                radius = color[i]
            if radius + color[i] > 255:
                radius = 255 - color[i]
            channel = blur_img[:, :, i]
            retval, minthresh = cv2.threshold(channel, color[i]-radius, 1, cv2.THRESH_BINARY)
            retval, maxthresh = cv2.threshold(channel, color[i]+radius, 1, cv2.THRESH_BINARY_INV)
            bw = numpy.multiply(bw, minthresh)
            bw = numpy.multiply(bw, maxthresh)
        #Now do some morphologies
        bw *= 255
        """
        close_radius = cv2.getTrackbarPos("close", "Processed image")
        close_kernel = numpy.array([close_radius, close_radius])

        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, close_kernel)"""
        return bw

class MouseListener():
    def __init__(self):
        self.done = False
        self.x_clicked = -1
        self.y_clicked = -1
    def onMouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_LBUTTONDOWN:
            print "Got mouse event:", x, ",", y
            self.x_clicked = x
            self.y_clicked = y
            self.done = True


def command_hand(limb, centroid, iksvc):
    limb_if = baxter_interface.Limb(limb)
    #Current orientation as a quaternion
    try:
        position = numpy.array(limb_if.endpoint_pose()['position'])
        orientation = numpy.array(limb_if.endpoint_pose()['orientation'])
    except KeyError:
        print "Didn't get position or orientation from limb interface"
        return
    #Rotation matrix
    R = tf.transformations.quaternion_matrix(orientation)
    R_inv = tf.transformations.inverse_matrix(R)
    p_centroid = numpy.array(list(centroid)+[0, 1])
    #Get goal direction
    direction = alpha*(R_inv.dot( p_centroid))

    #Add direction to current pose
    position = position + direction[0:3].flatten()
    print position, orientation
    desired_p = numpy.concatenate((position, orientation))
    print desired_p
    #Command
    ik_command.service_request(iksvc, desired_p, limb)
    

def nothing(args):
    pass

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    
    parser.add_argument('-m', '--method', choices=['color', 'edge', 'star'],
        required=False, help='which object detection method to use')

    """required.add_argument(
        '-f', '--folder', required=True, help='path to assets/ folder containing help images'
    )"""

    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    if args.method is None:
        args.method = 'star'

    print("Initializing node... ")
    rospy.init_node("baxter_object_tracker_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    
    cv2.namedWindow("Hand camera")
    cam = CameraSubscriber()
    cam.subscribe(limb)

    x_clicked = -1
    y_clicked = -1
    done = False
       
    print("Click on the object you would like to track, then press any key to continue.")
    ml = MouseListener()
    cv2.setMouseCallback("Hand camera", ml.onMouse)
    while not ml.done:
        cv2.waitKey(10)

    cv2.namedWindow("Processed image")
    cv2.namedWindow("Contours")

    detectMethod = None

        #TODO: Create sliders for morphological operators
    #elif args.method == 'rectangle':
    print "Starting image processor"
    cam.unsubscribe()
    imgproc = ProcessSubscriber(args.method, (ml.x_clicked, ml.y_clicked))
    imgproc.subscribe(limb)

    print "Press SPACE to begin hand servoing"
    while (not rospy.is_shutdown()) and (cv2.waitKey(100) != 32):
        pass

    iksvc, ns = ik_command.connect_service(limb)
    while not rospy.is_shutdown():
        print "Begin hand servoing"
        #cv2.waitKey(10) 
        #Move the hand 
        command_hand(limb, imgproc.centroid, iksvc)

if __name__ == "__main__":
    main()
