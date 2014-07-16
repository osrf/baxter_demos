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
import cv, cv2, cv_bridge
import numpy
from scipy.ndimage import label
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
        cv2.imshow("Hand camera", self.cur_img)
        cv2.waitKey(100)

    def get_data(self, data):
        img = cv_bridge.CvBridge().imgmsg_to_cv2(data)
        self.cur_img = img

class ProcessSubscriber(CameraSubscriber):
    def __init__(self, method, point, gamma = 1):
        self.gamma = gamma
        #TODO: write wrappers for edge and color detection
        cv2.createTrackbar("gamma", "Processed image", 80, 100, self.updateGamma)
        if method == 'edge':
            cv2.createTrackbar("threshold 1", "Processed image", 500, 2000, nothing)
            cv2.createTrackbar("threshold 2", "Processed image", 10, 2000, nothing)
            self.detectFunction = self.edgeDetect
        elif method == 'color':
            cv2.createTrackbar("blur", "Processed image", 12, 50, nothing)
            cv2.createTrackbar("radius", "Processed image", 6, 128, self.updateRadius)
            self.radius = 10
            cv2.createTrackbar("open", "Processed image", 4, 15, nothing)
            self.detectFunction = self.colorDetect

        elif method == 'star':
            maxSize = 45 # Maximum number of filters to apply?
            responseThreshold = 40 # higher = fewer features retrieved
            lineThresholdProjected = 15  # maximum ratio between Harris of responses. higher = eliminates more edges
            lineThresholdBinarized = 20 #maximum ratio between Harris of sizes. higher = more points
            cv2.createTrackbar("Response threshold", "Processed image", 20, 90, self.updateDetector)
            cv2.createTrackbar("Projected line threshold", "Processed image", 3, 30, self.updateDetector)
            cv2.createTrackbar("Binarized line threshold", "Processed image", 3, 30, self.updateDetector)
            self.detector = cv2.StarDetector(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized) 
            self.detectFunction = self.starDetect
        elif method == 'watershed':
            self.detectFunction = self.watershedDetect
            cv2.createTrackbar("blur", "Processed image", 4, 15, nothing)

        self.point = point
        self.centroid = point
        self.prev_img = None

    def updateRadius(self, r):
        self.radius = r

    def updateGamma(self, g):
        #g = cv2.getTrackbarPos("gamma", "Processed image")
        self.gamma = float(g)/100.0

    def updateDetector(self):
        maxSize = 45
        responseThreshold = cv2.getTrackbarPos("Response threshold", "Processed image")
        lineThresholdProjected = cv2.getTrackbarPos("Projected line threshold", "Processed image")
        lineThresholdBinarized = cv2.getTrackbarPos("Binarized line threshold", "Processed image")
        self.detector = cv2.StarDetector(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized) 

    def simpleFilter(self):
        #Very simple filter
        if self.prev_img is None:
            self.prev_img = self.cur_img
        return (self.gamma * self.cur_img + (1-self.gamma)*self.prev_img).astype(numpy.uint8)

    def callback(self, data):
        CameraSubscriber.get_data(self, data)
        #Process image
        cv2.imshow("Hand camera", self.cur_img)

        self.img = self.simpleFilter()

        self.processed = self.detectFunction(self.img)
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

        self.prev_img = self.img

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
        delta = 1000
        prev_compactness = sys.maxint
        for k in range(1, max(2, int(n/3))):
            print "Trying k-means with k="+str(k)+", n="+str(n)
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
        labels = labels.flatten()
        polys = {} #Dictionary to group the polygons
        for i in range(labels.shape[0]):
            polys.setdefault(labels[i], []).append(samples[i, :])
        
        #set this guy to all black
        gray[:] = 0 
        # Draw polygon around clusters
        # load these values from file because calculating this is godawful
        phi = (1.0+sqrt(5))/2.0
        hues = [ numpy.array([floor( (i*phi - floor(i*phi)) * 179), 255, 255]) for i in range(0, 100, 10)]
        
        colors = [tuple(cv2.cvtColor(hue.reshape((1, 1, 3)).astype(numpy.uint8), cv2.COLOR_HSV2BGR).flatten().tolist()) for hue in hues]
        for i in polys.keys():
            points = numpy.array(polys[i]).astype(int)

            """for point in points:
                # Draw clustered points in the same color
                cv2.circle(img=img, center=tuple(point.flatten().tolist()), radius=2, color=colors[i % len(colors)], thickness=-1)"""

            #Draw the convex hull of the points in the processed image
            hull = cv2.convexHull(points)
            hull = hull.reshape((hull.shape[0], 1, 2))
            cv2.fillConvexPoly(gray, hull, color=255)
        cv2.imshow("Hand camera", img)
        return gray

    def watershedDetect(self, img):
        # Do some preprocessing
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_radius = cv2.getTrackbarPos("blur", "Processed image")
        blur_radius = blur_radius*2-1
        if blur_radius > 0:
            gray = cv2.GaussianBlur(gray, (blur_radius, blur_radius), 0)
        # Blur/diffusion filter (?)
        # Get initial markers

        ret, markers = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
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
        thresh1 = cv2.getTrackbarPos('threshold 1', "Processed image")
        thresh2 = cv2.getTrackbarPos('threshold 2', "Processed image")
        canny = cv2.Canny(gray, thresh1, thresh2)
        return canny

    def colorDetect(self, img):
        #Blur the image to get rid of those annoying speckles
        blur_radius = cv2.getTrackbarPos("blur", "Processed image")
        blur_radius = blur_radius*2-1
        if blur_radius > 0:
            blur_img = cv2.GaussianBlur(img, (blur_radius, blur_radius), 0)
        else:
            blur_img = img

        #Get color of point in image
        blur_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV).astype(numpy.uint8)
        print self.point
        color = blur_img[self.point[1], self.point[0]]
        print color
        radius = self.radius
        #Grab the R, G, B channels as separate matrices
        #use cv2.threshold on each of them
        #AND the three images together
        bw = numpy.ones(img.shape[0:2], numpy.uint8)
        maxvals = [179, 255, 255]
        for i in [0]: #Just try hue
            minval = color[i] - radius
            maxval = color[i] + radius
            if radius > color[i]:
                minval = 0
            elif radius + color[i] > maxvals[i]:
                minval = color[i] - radius

            channel = blur_img[:, :, i]
            retval, minthresh = cv2.threshold(channel, minval, 255, cv2.THRESH_BINARY)
            retval, maxthresh = cv2.threshold(channel, maxval, 255, cv2.THRESH_BINARY_INV)
            bw = cv2.bitwise_and(bw, minthresh)
            bw = cv2.bitwise_and(bw, maxthresh)
        bw *= 255
        
        open_radius = cv2.getTrackbarPos("open", "Processed image")
        if open_radius != 0:
            open_kernel = numpy.array([open_radius, open_radius])

            bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, open_kernel, iterations = 2)
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
    
    parser.add_argument('-m', '--method', choices=['color', 'edge', 'star', 'watershed'],
        required=False, help='which object detection method to use')

    """required.add_argument(
        '-f', '--folder', required=True, help='path to assets/ folder containing help images'
    )"""

    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    if args.method is None:
        args.method = 'watershed'

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

    while not rospy.is_shutdown():
        cv2.waitKey(10)


    """print "Press SPACE to begin hand servoing"
    while (not rospy.is_shutdown()) and (cv2.waitKey(100) != 32):
        pass

    iksvc, ns = ik_command.connect_service(limb)
    while not rospy.is_shutdown():
        print "Begin hand servoing"
        #cv2.waitKey(10) 
        #Move the hand 
        command_hand(limb, imgproc.centroid, iksvc)"""

if __name__ == "__main__":
    main()
