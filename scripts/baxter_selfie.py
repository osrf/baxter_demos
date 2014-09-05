#!/usr/bin/env python

import rospy
import argparse
import baxter_interface
from baxter_interface import CHECK_VERSION
import common
import cv, cv2, cv_bridge
import numpy
import datetime
import yaml

from sensor_msgs.msg import Image

img = None
demos_path = rospy.get_param('baxter_demos_folder')
print demos_path
with open(demos_path+'/config/selfie.yaml', 'r') as f:
    print "loading yaml"
    params = yaml.load(f)

def main():
    global img
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('-n', '--name', required=False,
        help='filename to save selfie to')
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    if args.name is None:
        args.name = datetime.datetime.now().isoformat()+".jpg"

    rospy.init_node("baxter_selfie")
 
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    limb_if = baxter_interface.Limb(limb)
    
    joint_names = params['joint_names']
    joint_positions = params['joint_positions']

    selfie_pose = dict(zip(joint_names, joint_positions )) # TODO
   
    # Load up some params

    scale_factor  = params['scale_factor']
    min_neighbors = params['min_neighbors']
    path = demos_path + "/" + params['path']

    # load a cascade classifier
    face_cascade = cv2.CascadeClassifier(demos_path + "/config/haarcascade_frontalface_default.xml")

    def callback(data):
        global img
        img = cv_bridge.CvBridge().imgmsg_to_cv2(data)

    # Subscribe to the relevant nodes
    cam_sub = rospy.Subscriber("/cameras/"+limb+"_hand_camera/image", Image, callback)
    screen_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)


    # Move to the selfie pose
    limb_if.move_to_joint_positions(selfie_pose, timeout=10.0)


    cv2.namedWindow("Hand camera")
    # Display hand camera image on screen and wait until face is detected
    while not rospy.is_shutdown():
        if img is None:
            continue
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgra8")
        screen_pub.publish(msg)
        cv2.imshow("Hand camera", img)
        cv2.waitKey(100)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scale_factor, min_neighbors)
        if len(faces) > 0:
            print "Found face"
            break

    # Save image
    cv2.imwrite(path+args.name, img)
    print "Saved selfie to", path+args.name

if __name__ == "__main__":
    main()
