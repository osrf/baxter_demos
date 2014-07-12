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
    def __init__(self, detectFunction, point):
        self.detectFunction = detectFunction
        self.point = point
        self.centroid = point
    
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
 

def edgeDetect(img, point = None):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh1 = cv2.getTrackbarPos('threshold 1', "Processed image")
    thresh2 = cv2.getTrackbarPos('threshold 2', "Processed image")
    canny = cv2.Canny(gray, thresh1, thresh2)
    return canny

def colorDetect(img, point):
    #Blur the image to get rid of those annoying speckles
    blur_radius = cv2.getTrackbarPos("blur", "Processed image")
    blur_radius = blur_radius*2-1
    blur_img = cv2.GaussianBlur(img, (blur_radius, blur_radius), 0)
    #Get color of point in image
    blur_img = img
    color = blur_img[point[0], point[1]]
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
    """open_radius = cv2.getTrackbarPos("open", "Processed image")
    open_kernel = numpy.array([open_radius, open_radius])

    close_radius = cv2.getTrackbarPos("close", "Processed image")
    close_kernel = numpy.array([close_radius, close_radius])

    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, open_kernel)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, close_kernel)"""
    return bw

"""
def cornerDetect(img, point = None):"""

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
       
    print("Click on the object you would like to track, then press any key to continue.")
    ml = MouseListener()
    cv2.setMouseCallback("Hand camera", ml.onMouse)
    while not ml.done:
        cv2.waitKey(10)

    cv2.namedWindow("Processed image")
    cv2.namedWindow("Contours")

    detectMethod = None
    if args.method == 'edge':
        cv2.createTrackbar("threshold 1", "Processed image", 500, 2000, nothing)
        cv2.createTrackbar("threshold 2", "Processed image", 10, 2000, nothing)
        detectMethod = edgeDetect
    elif args.method == 'color':
        cv2.createTrackbar("blur", "Processed image", 2, 15, nothing)
        cv2.createTrackbar("radius", "Processed image", 10, 128, nothing)
        cv2.createTrackbar("open", "Processed image", 3, 15, nothing)
        cv2.createTrackbar("close", "Processed image", 3, 15, nothing)

        #TODO: Create sliders for morphological operators
        detectMethod = colorDetect
    #elif args.method == 'corner':
    print "Starting image processor"
    cam.unsubscribe()
    imgproc = ProcessSubscriber(detectMethod, (ml.x_clicked, ml.y_clicked))
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
