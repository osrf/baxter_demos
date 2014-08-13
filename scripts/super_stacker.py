#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import yaml
import os
import argparse
import numpy

from visual_servo import VisualCommand
import common
import ik_command

from geometry_msgs.msg import Pose, PoseArray

config_folder = rospy.get_param('object_tracker/config_folder')

with open(config_folder+'servo_to_object.yaml', 'r') as f:
    params = yaml.load(f)

class DepthCaller:
    def __init__(self, limb):
        self.done = False
        self.limb = limb
        self.object_height = params['object_height']
        self.depth_handler = rospy.Subscriber("object_tracker/"+limb+"/goal_poses", PoseArray, self.depth_callback)

    def depth_callback(self, data):
        print "Estimating depth"
        self.object_poses = []
        for pose in data.poses:
            p = [pose.position.x, pose.position.y, pose.position.z + self.object_height]+\
                [pose.orientation.x, pose.orientation.y, pose.orientation.z,\
                pose.orientation.w]
            self.object_poses.append(p)
        #print p

        self.done = True
        print "unregistering"
        self.depth_handler.unregister()

def incrementPoseZ(pose, inc):
    pos = numpy.array(pose[0:3])
    pos += numpy.array((0, 0, inc))
    pose = numpy.concatenate( (pos, pose[3:7]) )
    return pose.tolist()

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )

    args = parser.parse_args(rospy.myargv()[1:])
    print args
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("super_stacker_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    #Calibrate gripper
    gripper_if = baxter_interface.Gripper(limb)
    if not gripper_if.calibrated():
        print "Calibrating gripper"
        gripper_if.calibrate()

    limbInterface = baxter_interface.Limb(limb)

    iksvc, ns = ik_command.connect_service(limb)
    rate = rospy.Rate(params['rate'])

    # Get goal poses of each object in the scene
    dc = DepthCaller(limb)

    # Subscribe to estimate_depth
    # Move to pose published by estimate_depth
    while (not dc.done) and (not rospy.is_shutdown()):
        rate.sleep()
        #pass

    
    print "Start visual servoing to first object"
    
    # Subscribe to object_finder and start visual servoing/grasping
    vc = VisualCommand(iksvc, limb)
    vc.subscribe()

    # The stack pose is the pose of the smallest object with a z-offset
    # accounting for the height of the object (this assumption makes
    # it really important that the segmentation is accurate)
    #stack_pose = [0.594676466827,     -0.296644499519,     -0.0322744943164,     0.971805911045,     -0.22637170004,     0.065946440385,     0.000437813100735]
    stack_pose = dc.object_poses[-1]
    stack_pose = incrementPoseZ(stack_pose, dc.object_height)
    dc.object_poses.pop(len(dc.object_poses)-1)

    for pose in dc.object_poses[:len(dc.object_poses)]:

        ik_command.service_request(iksvc, pose, limb, blocking=True)

        while (not vc.done) and (not rospy.is_shutdown()):
            rate.sleep()

        ik_command.service_request(iksvc, stack_pose, limb, blocking=True)
        #limbInterface.move_to_joint_positions(stack_pose)

        # Let go
        gripper_if.open(block=True)

        stack_pose = incrementPoseZ(pose, dc.object_height)


if __name__ == "__main__":
    main()
