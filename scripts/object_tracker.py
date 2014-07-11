#!/usr/bin/env python

"""Simple Object Tracker by Jackie Kay (jackie@osrfoundation.org)
Select one of several object recognition methods and an object in Baxter's hand camera to track.
Baxter will try to keep the camera aligned with the object centroid.
"""

import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import common
import cv2, cv_bridge
import numpy
from sensor_msgs.msg import (
    Image,
)

class CameraSubscriber:
    def subscribe(self, limb):
        self.limb = limb
        self.handler = rospy.Subscriber("/cameras/"+limb+"_hand_camera/image", Image, self.callback)

    def unsubscribe(self):
        self.handler.unregister()

    def callback(self, data):
        self.get_data(data)
        cv2.waitKey(100)

    def get_data(self, data):
        img = cv_bridge.CvBridge().imgmsg_to_cv2(data)
        self.img = img
        cv2.imshow("Hand camera", self.img)

class ProcessSubscriber(CameraSubscriber):
    def __init__(self, detectFunction, point):
        self.detectFunction = detectFunction
        self.point = point
    
    def callback(self, data):
        CameraSubscriber.get_data(self, data)
        #Process image
        self.processed = self.detectFunction(self.img, self.point)
        cv2.imshow("Processed image", self.processed)
        
        #Find the contour associated with self.point
        contour_img = self.processed.copy()
        contours, hierarchy = cv2.findContours(contour_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None:
            return
            
        cv2.drawContours(contour_img, contours, -1, (255, 255, 255))

        #Might want a more sophisticated test, e.g. start with the innermost contours
        for contour in contours:
            if cv2.pointPolygonTest(contour, self.point, False) > 0:
                print "Found contour encircling desired point"
                break
        #Find the centroid of this contour
        centroid = [int(numpy.mean(contour[:, :, 0])), int(numpy.mean(contour[:, :, 1])) ]
        print "Found centroid:", centroid
        cv2.circle(img=contour_img, center=tuple(centroid), radius=5, color=(255, 255, 255), thickness=-1)
        cv2.imshow("Contours", contour_img)
        cv2.waitKey(300)

def edgeDetect(img, point = None):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh1 = cv2.getTrackbarPos('threshold 1', "Processed image")
    thresh2 = cv2.getTrackbarPos('threshold 2', "Processed image")
    canny = cv2.Canny(gray, thresh1, thresh2)
    return canny

def colorDetect(img, point):
    #Get color of point in image
    color = img[point[0], point[1]]
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
        channel = img[:, :, i]
        retval, minthresh = cv2.threshold(channel, color[i]-radius, 1, cv2.THRESH_BINARY_INV)
        retval, maxthresh = cv2.threshold(channel, color[i]+radius, 1, cv2.THRESH_BINARY_INV)
        bw = numpy.multiply(bw, minthresh)
        bw = numpy.multiply(bw, maxthresh)
    return 255 * bw

"""
def cornerDetect(img, point = None):"""

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
    
    parser.add_argument('-m', '--method', choices=['color', 'edge', 'corner'],
        required=False, help='which object detection method to use')

    """required.add_argument(
        '-f', '--folder', required=True, help='path to assets/ folder containing help images'
    )"""

    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    if args.method is None:
        args.method = 'edge'

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

    def onMouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_LBUTTONDOWN:
            print "Got mouse event:", x, ",", y
            x_clicked = x
            y_clicked = y
            done = True
        
    print("Click on the object you would like to track, then press any key to continue.")
    cv2.setMouseCallback("Hand camera", onMouse)
    cv2.waitKey()

    cv2.namedWindow("Processed image")
    cv2.namedWindow("Contours")

    detectMethod = None
    if args.method == 'edge':
        cv2.createTrackbar("threshold 1", "Processed image", 500, 2000, nothing)
        cv2.createTrackbar("threshold 2", "Processed image", 10, 2000, nothing)
        detectMethod = edgeDetect
    elif args.method == 'color':
        cv2.createTrackbar("radius", "Processed image", 10, 128, nothing)
        detectMethod = colorDetect
    #elif args.method == 'corner':
    print "Starting image processor"
    cam.unsubscribe()
    imgproc = ProcessSubscriber(detectMethod, (x_clicked, y_clicked))
    imgproc.subscribe(limb)
    while not rospy.is_shutdown():
        cv2.waitKey(10) 

if __name__ == "__main__":
    main()
