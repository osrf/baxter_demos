#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import os
import argparse
import yaml

from visual_servo import VisualCommand
import common
import ik_command

from geometry_msgs.msg import Pose, PoseArray


config_folder = rospy.get_param('object_tracker/config_folder')

with open(config_folder+'servo_to_object.yaml', 'r') as f:
    params = yaml.load(f)

class DepthCaller:
    def __init__(self, limb, iksvc):
        self.done = False
        self.iksvc = iksvc
        self.limb = limb

        self.depth_handler = rospy.Subscriber("object_tracker/"+limb+"/goal_poses", PoseArray, self.depth_callback)

    def depth_callback(self, data):
        print "Estimating depth"
        # Get multiple poses. For now, just choose the first one, which corresponds to the biggest object
        if len(data.poses) <= 0:
            print "no poses"
            return
        pose = data.poses[0]
        p = [pose.position.x, pose.position.y, pose.position.z]+[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        ik_command.service_request(self.iksvc, p, self.limb, blocking=True)
        print p

        self.done = True
        print "unregistering"
        self.depth_handler.unregister()


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    required.add_argument(
        '-f', '--folder', help='path to assets/ folder containing help images'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    print args
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("servo_to_object_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    #Calibrate gripper
    gripper_if = baxter_interface.Gripper(limb)
    if not gripper_if.calibrated():
        print "Calibrating gripper"
        gripper_if.calibrate()

    if args.folder is None:
        args.folder = "/home/jackie/ros_ws/src/baxter_demos/assets/"
    # First, get start and end configurations from the user
    filenames = ["getpoint1.png", "getpoint2.png", "executing_grasp.png"]
    filenames = [args.folder+filename for filename in filenames]
    
    for filename in filenames:
        if not os.access(filename, os.R_OK):
            rospy.logerr("Cannot read file at '%s'" % (filename,))
            return 1

    limbInterface = baxter_interface.Limb(limb)

    #rospy.on_shutdown(display)

    common.send_image(filenames[0])
    #Get the current position
    buttonpress = common.ButtonListener()
    buttonpress.subscribe("/robot/digital_io/"+limb+"_lower_button/state")
    points = []

    while not buttonpress.pressed:
        rospy.sleep(params['button_rate'])
    
    points.append( buttonpress.getButtonPress(limbInterface))
    print "Got first position:"
    print points[0]

    buttonpress.pressed = False

    common.send_image(filenames[1])

    while not buttonpress.pressed:
        rospy.sleep(params['button_rate'])
 
    points.append( buttonpress.getButtonPress(limbInterface))
    print "Got second position:"
    print points[1]

    common.send_image(filenames[2])

    # Command start configuration

    limbInterface.move_to_joint_positions(points[0])

    iksvc, ns = ik_command.connect_service(limb)

    rate = rospy.Rate(params['rate'])
    dc = DepthCaller(limb, iksvc)

    # Subscribe to estimate_depth
    # Move to pose published by estimate_depth
    while (not dc.done) and (not rospy.is_shutdown()):
        rate.sleep()

    print "Starting visual servoing"
    
    # Subscribe to object_finder and start visual servoing/grasping
    vc = VisualCommand(iksvc, limb)
    vc.subscribe()

    while (not vc.done) and (not rospy.is_shutdown()):
        rate.sleep()

    limbInterface.move_to_joint_positions(points[1])

    # Let go
    gripper_if.open()

if __name__ == "__main__":
    main()
