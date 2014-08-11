#!/usr/bin/env python

"""Simple Object Tracker by Jackie Kay (jackie@osrfoundation.org)
User selects a detection method and an object in Baxter's hand camera view.
This node returns an array of geometric data about all the matching objects
in the scene.
Geometric data includes the centroid of the desired object in the camera frame
and the axis through an object with a straight edge.
"""

import sys
import argparse
import yaml
from math import sqrt, floor, pi
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import common
import cv, cv2, cv_bridge
import numpy
from scipy.ndimage import label
import tf

from sensor_msgs.msg import Image
from baxter_demos.msg import BlobInfoArray
from baxter_demos.msg import BlobInfo
from geometry_msgs.msg import(
    Point,
    Polygon
)

node_name = "object_finder"
config_folder = rospy.get_param('object_tracker/config_folder')

with open(config_folder+'object_finder.yaml', 'r') as f:
    params = yaml.load(f)

raw_win, processed_win, edge_win = ["raw_win", "processed_win", "edge_win"]
raw_win = params[raw_win]
processed_win = params[processed_win]
edge_win = params[edge_win]

"""def getHoughArgs():
    names = ["rho", "theta", "threshold", "minLineLength", "maxLineGap"]
    node_full_name = node_name+"/"
    return [rospy.get_param(node_full_name+name) for name in names]"""

class CameraSubscriber:
    """Subscribe to the hand camera image and convert the message to an OpenCV-
    friendly format"""

    def __init__(self):
        self.cv_wait = params['rate']
       
    def subscribe(self, topic="/cameras/right_hand_camera/image"):
        self.handler_sub = rospy.Subscriber(topic, Image, self.callback)
        self.cur_img = None

    def unsubscribe(self):
        self.handler_sub.unregister()

    def callback(self, data):
        self.get_data(data)
        
    def get_data(self, data):
        img = cv_bridge.CvBridge().imgmsg_to_cv2(data)
        self.cur_img = img

class ObjectFinder(CameraSubscriber):
    def __init__(self, method, point):

        self.cv_wait = params['rate']

        cv2.createTrackbar("gamma", processed_win, params['gamma'],
                           params['gamma_max'], self.updateGamma)

        cv2.createTrackbar("threshold 1", edge_win, params['thresh1'],
                           params['thresh_max'], nothing)
        cv2.createTrackbar("threshold 2", edge_win, params['thresh2'],
                           params['thresh_max'], nothing)

        if method == 'edge':
           self.detectFunction = self.edgeDetect

        elif method == 'color':
            cv2.createTrackbar("blur", processed_win, params['blur'],
                               params['blur_max'], nothing)
            cv2.createTrackbar("radius", processed_win, params['radius'],
                               params['radius_max'], nothing)
            cv2.createTrackbar("open", processed_win, params['open'],
                               params['open_max'], nothing)
            self.detectFunction = self.colorDetect
            self.color = None

            self.houghArgs = [params['rho'], params['theta'], params['threshold'],
                              params['minLineLength'], params['maxLineGap']]

        elif method == 'star':
            
            cv2.createTrackbar("Response threshold", processed_win,
                                params['response'], params['response_max'],
                                self.updateDetector)
            cv2.createTrackbar("Projected line threshold", processed_win,
                                params['projected'], params['projected_max'],
                                self.updateDetector)
            cv2.createTrackbar("Binarized line threshold", processed_win,
                                params['binarized'], params['binarized_max'],
                                self.updateDetector)
            self.detector = cv2.StarDetector(params['maxSize'],
                                    params['responseThreshold'],
                                    params['lineThresholdProjected'],
                                    params['lineThresholdBinarized']) 
            self.detectFunction = self.starDetect

        elif method == 'watershed':
            self.detectFunction = self.watershedDetect
            cv2.createTrackbar("blur", processed_win, params['blur'],
                               params['blur_max'], nothing)

        self.point = point
        self.centroids = [] 
        self.axes = []
        #self.prev_axis = None
        self.prev_img = None
        self.processed = None
        self.canny = None

    def publish(self, limb):
        topic = "object_tracker/"+limb+"/centroid"

        self.handler_pub = rospy.Publisher(topic, BlobInfo)
        self.pub_rate = rospy.Rate(params['rate'])

    def updateGamma(self, g):
        self.gamma = float(g)/100.0

    def updateDetector(self):
        maxSize = params['maxSize']
        responseThreshold = cv2.getTrackbarPos("Response threshold",
                                                processed_win)
        lineThresholdProjected = cv2.getTrackbarPos("Projected line threshold",
                                                    processed_win)
        lineThresholdBinarized = cv2.getTrackbarPos("Binarized line threshold",
                                                    processed_win)
        self.detector = cv2.StarDetector(maxSize, responseThreshold,
                                lineThresholdProjected, lineThresholdBinarized) 

    def simpleFilter(self):
        #Very simple filter
        if self.prev_img is None:
            self.prev_img = self.cur_img
        if self.prev_img.shape != self.cur_img.shape:
            return self.cur_img
        return (self.gamma * self.cur_img + (1-self.gamma)*self.prev_img)\
                .astype(numpy.uint8)

    def getEncirclingContour(self, contours):
        for contour in contours:
            if cv2.pointPolygonTest(contour, self.point, False) > 0:
                return contour

    def getLargestContour(self, contours):
        maxpair = (None, 0)
        if len(contours) == 0:
            raise Exception("Got no contours in getLargestContour")
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > maxpair[1]:
                maxpair = (contour, area)
        return maxpair[0]

    def callback(self, data):
        CameraSubscriber.get_data(self, data)

        #Process image
        self.img = self.simpleFilter()
        self.processed = self.detectFunction(self.img)
        
        # Find the contour associated with self.point
        contour_img = self.processed.copy()
        contours, hierarchy = cv2.findContours(contour_img, cv2.RETR_LIST,
                                               cv2.CHAIN_APPROX_SIMPLE)
        if contours is None or len(contours) == 0:
            rospy.loginfo( "no contours found" )
            self.centroids = []
            return
            
        cv2.drawContours(contour_img, contours, -1, (255, 255, 255))
        contour_img = cv2.cvtColor(contour_img, cv2.COLOR_GRAY2BGR)
        self.centroids = []
        self.axes = []

        # Sort contours by area
        contours.sort(key=cv2.contourArea, reverse=True)
        for contour in contours:
            # Find the centroid of this contour
            moments = cv2.moments(contour)
            if moments['m00'] == 0:
                continue
            centroid = ( int(moments['m10']/moments['m00'] ),
                              int(moments['m01']/moments['m00']), 0 )
            self.centroids.append( centroid )
     
            self.canny = self.edgeDetect(contour_img)
            axis = self.getObjectAxes(self.canny, contour)
            self.axes.append( axis )

            # Draw the detected object
            if axis is not None:
                cv2.line(self.cur_img, tuple(axis[0:2]),
                                       tuple(axis[3:5]), (0, 255, 0), 2)
            """    self.prev_axis = self.axis
            elif self.prev_axis is not None:
                cv2.line(self.cur_img, tuple(prev_axis[0:2]),
                                       tuple(prev_axis[3:5]), (0, 255, 0), 2)"""


            cv2.circle(img=self.cur_img, center=centroid[0:2], radius=2,
                       color=(0, 255, 0), thickness=-1)
        
        self.prev_img = self.img

    def updatePoint(self, event, x, y, flags, param):
        #blur the image and get a new color
        if event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_LBUTTONDOWN:
            blur_radius = cv2.getTrackbarPos("blur", processed_win)
            point = (x, y)
            self.point = point
            if self.cur_img is None:
                return
            blur_img = common.blurImage(self.cur_img, blur_radius)
            self.color = blur_img[self.point[1], self.point[0]]
            self.axes = []

    def getObjectAxes(self, img, contour):
        # Get the bounding rectangle around the detected object
        rect = cv2.boundingRect(contour)
        
        # Calculate a border around the bounding rectangle
        pad = int((rect[2]+rect[3])*params['pad'])

        if rect[0] < pad:  x = 0
        else: x = rect[0]-pad

        if rect[1] < pad:  y = 0
        else:  y = rect[1]-pad

        p1 = (x, y)

        x, y = rect[0]+rect[2]+pad, rect[1]+rect[3]+pad
        
        if x > img.shape[1]:  x = img.shape[1] - 1
        if y > img.shape[0]:  y = img.shape[0] - 1

        p2 = (x, y)

        cv2.rectangle(self.cur_img, p2, p1, (0, 255, 0), 3)
        
        # Extract the region of interest around the detected object
        subimg = img[p1[1]:p2[1], p1[0]:p2[0]]
       
        # Hough line detection
        lines = cv2.HoughLinesP(subimg, *self.houghArgs)
        if lines is None:
            rospy.loginfo( "no lines found" )
            return None

        lines = lines.reshape(lines.shape[1], lines.shape[2])

        # Calculate the lengths of each line
        lengths = numpy.square(lines[:, 0]-lines[:, 2]) +\
                  numpy.square(lines[:, 1]-lines[:, 3])
        lines = numpy.hstack((lines, lengths.reshape((lengths.shape[0], 1)) ))
        lines = lines[lines[:,4].argsort()]
        lines = lines[::-1] #Reverse the sorted array
        lines = lines[:, 0:4]

        bestline = lines[0] #Get the longest line
        bestline += numpy.tile(numpy.array(p1), 2)

        axis = numpy.hstack( (bestline[0:2], 0, bestline[2:4], 0) )
        print "axis found:", axis
        return axis
        
    def starDetect(self, img):
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
        delta = params['delta']
        prev_compactness = sys.maxint
        for k in range(1, max(2, int(n/3))):
            print "Trying k-means with k="+str(k)+", n="+str(n)
            max_iter = params['max_iter']
            epsilon = params['epsilon']
            attempts = params['attempts']
            compactness, labels, centers = cv2.kmeans(samples, k,
                                 (cv2.TERM_CRITERIA_EPS +
                                 cv2.TERM_CRITERIA_MAX_ITER, max_iter, epsilon),
                                 attempts, cv2.KMEANS_RANDOM_CENTERS)
            if prev_compactness - compactness < delta:
                if k > 1:
                    k-=1
                break
            prev_compactness = compactness
        # Put the output of k-means into a convenient dictionary
        labels = labels.flatten()
        polys = {} #Dictionary to group the polygons
        for i in range(labels.shape[0]):
            polys.setdefault(labels[i], []).append(samples[i, :])
        
        #set this guy to all black
        gray[:] = 0 
        # Draw polygon around clusters
        # load these values from file because calculating this is godawful
        phi = (1.0+sqrt(5))/2.0
        hues = [ numpy.array([floor( (i*phi - floor(i*phi)) * 179), 255, 255])
                 for i in range(0, 100, 10)]
        
        colors = [tuple(cv2.cvtColor(hue.reshape((1, 1, 3)).astype(numpy.uint8),
                  cv2.COLOR_HSV2BGR).flatten().tolist()) for hue in hues]
        for i in polys.keys():
            points = numpy.array(polys[i]).astype(int)

            #Draw the convex hull of the points in the processed image
            hull = cv2.convexHull(points)
            hull = hull.reshape((hull.shape[0], 1, 2))
            cv2.fillConvexPoly(gray, hull, color=255)
        return gray

    def watershedDetect(self, img):
        # Do some preprocessing
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_radius = cv2.getTrackbarPos("blur", processed_win)
        blur_radius = blur_radius*2-1
        if blur_radius > 0:
            gray = cv2.GaussianBlur(gray, (blur_radius, blur_radius), 0)
        # Blur/diffusion filter (?)
        # Get initial markers

        ret, markers = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV +
                                                   cv2.THRESH_OTSU)
        kernel_size = 5
        kernel = numpy.ones((kernel_size,kernel_size),numpy.uint8)

        markers = cv2.morphologyEx(markers, cv2.MORPH_OPEN,kernel, iterations = 3)

        border = cv2.dilate(markers, kernel,iterations=3)
        border = border - cv2.erode(border, kernel, iterations=1)
        retval, border = cv2.threshold(border, 0, 255, cv2.THRESH_OTSU)
        border = cv2.morphologyEx(border, cv2.MORPH_CLOSE,kernel, iterations = 3)

        dt = cv2.distanceTransform(markers, 2, 3)
        dt = ((dt - dt.min()) / (dt.max() - dt.min()) * 255).astype(numpy.uint8)
        _, dt = cv2.threshold(dt, 180, 255, cv2.THRESH_BINARY)
        lbl, ncc = label(dt)
        lbl = lbl * (255/ncc)
        lbl[border == 255] = 255

        # segment
        markers = lbl.astype(numpy.int32)
        if img.dtype != numpy.uint8:
            img = img.astype(numpy.uint8)
        if img.shape[2] > 3:
            img = img[:, :, 0:3]
        cv2.watershed(img, markers)
        # Fill image

        result = markers
        result[markers == -1] = 0
        result = result.astype(numpy.uint8)
        return result

    def edgeDetect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh1 = cv2.getTrackbarPos('threshold 1', edge_win)
        thresh2 = cv2.getTrackbarPos('threshold 2', edge_win)
        canny = cv2.Canny(gray, thresh1, thresh2)
        return canny

    def colorDetect(self, img):
        #Blur the image to get rid of those annoying speckles
        blur_radius = cv2.getTrackbarPos("blur", processed_win)
        radius = cv2.getTrackbarPos("radius", processed_win)
        open_radius = cv2.getTrackbarPos("open", processed_win)
        blur_img = common.blurImage(img, blur_radius)

        if self.color == None:
            self.color = blur_img[self.point[1], self.point[0]]

        return common.colorSegmentation(blur_img, blur_radius, radius,
                                        open_radius, self.color)

def cleanup():
    cv2.destroyAllWindows()

def nothing(args):
    pass

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=False, choices=['left', 'right'],
        help='which limb to send joint trajectory'
    )
    
    parser.add_argument('-m', '--method',
                        choices=['color', 'edge', 'star', 'watershed'],
                        required=False, help='which detection method to use')
    parser.add_argument('-t', '--topic', required=False,
                        help='which image topic to listen on')

    args = parser.parse_args(rospy.myargv()[1:])
    if args.limb is None:
        limb = "right"
    else:
        limb = args.limb
    if args.method is None:
        args.method = 'color'
    if args.topic is None:
        args.topic = "/cameras/"+limb+"_hand_camera/image"

    print args
    print("Initializing node... ")
    rospy.init_node(node_name)

    rospy.on_shutdown(cleanup)

    baxter_cams = ["/cameras/right_hand_camera/image",
                    "/cameras/left_hand_camera/image",
                    "/cameras/head_camera/image"]
    if args.topic in baxter_cams:
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        print("Enabling robot... ")
        rs.enable()
    
    cv2.namedWindow(raw_win)
    cam = CameraSubscriber()
    cam.subscribe(args.topic)

    rospy.loginfo( "Click on the object you would like to track, then press\
                    any key to continue." )
    ml = common.MouseListener()

    if "object_finder_test" in args.topic:
        # Hardcoded position
        point = (322, 141)
        
    else:
        cv2.setMouseCallback(raw_win, ml.onMouse)
        while not ml.done:
            if cam.cur_img is not None:
                cv2.imshow(raw_win, cam.cur_img)

            cv2.waitKey(cam.cv_wait)
        point = (ml.x_clicked, ml.y_clicked)
    detectMethod = None

    cam.unsubscribe()

    cv2.namedWindow(processed_win)
    if cam.cur_img is not None:
        cv2.imshow(processed_win, numpy.zeros((cam.cur_img.shape)))

    print "Starting image processor"
    imgproc = ObjectFinder(args.method, point)
    imgproc.subscribe(args.topic)
    imgproc.publish(limb)
    cv2.namedWindow(edge_win)
    cv2.setMouseCallback(raw_win, imgproc.updatePoint)

    while not rospy.is_shutdown():
        blobArray = []
        
        for centroid, axis in zip(imgproc.centroids, imgproc.axes):
            blob = BlobInfo()
            centroid = Point(*centroid)
            blob.centroid = centroid
            if axis is None:
                axis = -1*numpy.ones(6)
            blob.axis = Polygon([Point(*axis[0:3].tolist()),
                                 Point(*axis[3:6].tolist())])
            blobArray.append(blob)

        msg = BlobInfoArray()
        msg.blobs = blobArray
        imgproc.handler_pub.publish(msg)

        if imgproc.cur_img is not None:
            cv2.imshow(raw_win, imgproc.cur_img)

        if imgproc.processed is not None:
            cv2.imshow(processed_win, imgproc.processed)
        if imgproc.canny is not None:
            cv2.imshow(edge_win, imgproc.canny)
        cv2.waitKey(imgproc.cv_wait)

if __name__ == "__main__":
    main()
