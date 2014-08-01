#!/usr/bin/env python

import os
import sys
import rospy
import cv, cv2, cv_bridge
import numpy
from baxter_demos.msg import BlobInfo
from sensor_msgs.msg import Image

"""Script to test functionality of object_finder color segmentation
User needs to manually run object_finder and subscribe to topic name specified here
"""
global centroid, axis
centroid = None
axis = None
img_path = "assets/block_imgs/"
topic_name = "object_finder_test"

img_names = os.listdir(img_path)

rospy.init_node("color_seg_test")

cv2.namedWindow("Test image")
for img_name in img_names:
    # publish image on test_img topic
    color_img = cv2.imread(img_path+img_name)
    
    color_msg = cv_bridge.CvBridge().cv2_to_imgmsg(color_img, "rgb8")
    img_pub = rospy.Publisher(topic_name, Image)

    # load and display segmented image
    #seg_img = cv2.imread(img_path+img_name)
    cv2.waitKey(100)

    def callback(data):
        global centroid, axis
        # get the centroid and axis found in object_finder
        centroid = (data.centroid.x, data.centroid.y)
        axis = [[data.axis.points[i].x, data.axis.points[i].y] for i in range(2)]

    object_sub = rospy.Subscriber("object_tracker/right/centroid", BlobInfo, callback)

    while not rospy.is_shutdown():
        img_pub.publish(color_msg)
        cv2.imshow("Test image", color_img)
        #Press space when satisfied (user might mess with params in other window)

        if cv2.waitKey(100) == 32 and centroid is not None and axis is not None:
            break
    
    # Process the segmented image a little    
    # get the shape containing the finding contours in the segmented pic
    """gray = cv2.cvtColor(seg_img, cv2.COLOR_RGB2GRAY)
    retval, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    maxpair = (None, 0)
    if len(contours) == 0:
        raise Exception("Got no contours in getLargestContour")
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > maxpair[1]:
            maxpair = (contour, area)
    encircler = maxpair[0]

    if encircler is None:
        print("Test failed: couldn't find encircling contour in segmented image")
        continue
        
    # Get centroid of encircler   
    moments = cv2.moments(encircler)
    expected_centroid = ( int(moments['m10']/moments['m00'] ),
                      int(moments['m01']/moments['m00']))
 
    print "Expected centroid:", expected_centroid
    print "Test centroid:", centroid
    distance = numpy.linalg.norm(numpy.array(centroid)-numpy.array(expected_centroid))
    print "Centroid error:", distance"""
    
