#!/usr/bin/env python

"""servo_3d.py:
Wait for the object_tracker/goal_pos(e) topic to be published (from estimate_depth or stereo)
Command a trajectory to the goal position
Invoke visual_servo (as a standalone node or as a class?) 
"""

import sys
import argparse

import rospy
import baxter_interface

import cv, cv2, cv_bridge
import numpy
import tf

import common
import ik_command

from geometry_msgs.msg import Pose

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
    limb = args.limb

    iksvc, ns = ik_command.connect_service(limb)

    rospy.init_node("servo_3d")
 
    done = False

    def servo_callback(data):
        p = [data.position.x, data.position.y, data.position.z]+[data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        print p
        ik_command.service_request(iksvc, p, limb, blocking=True)
        done = True

    # Subscribe to object_tracker/goal_pos
    # limb...?
    goal_sub = rospy.Subscriber("object_tracker/"+limb+"/goal_pose", Pose, servo_callback)
    
    rate = rospy.Rate(100)
    while not done and not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
