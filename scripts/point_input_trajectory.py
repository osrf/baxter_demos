#!/usr/bin/env python

"""Program to get two configuration points from the user, plot a trajectory
between them, and servo Baxter's arm in that trajectory indefinitely.
Based on joint_trajectory_client.py in the Rethink baxter_examples repo.
Code by Jackie Kay (jackie@osrfoundation.org)
"""
#Remember to run the joint trajectory action server:
#$ rosrun baxter_interface joint_trajectory_action_server.py 

import argparse
import sys
import os

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import common

import baxter_interface

from baxter_interface import CHECK_VERSION

import baxter_interface.digital_io as DIO



def main():
    """RSDK Joint Trajectory Example: Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.

    Make sure to start the joint_trajectory_action_server.py
    first.

    Get two points from the user.

    Use the Joint Trajectory Action Server to find a path between them.
    
    Repeat the trajectory until program is stopped.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    required.add_argument(
        '-f', '--folder', required=True, help='path to assets/ folder containing help images'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    print args
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("point_input_trajectory_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    traj = common.Trajectory(limb)
    rospy.on_shutdown(traj.stop)

    filenames = ["getpoint1.png", "getpoint2.png", "executing.png"]
    filenames = [args.folder+filename for filename in filenames]
    
    for filename in filenames:
        if not os.access(filename, os.R_OK):
            rospy.logerr("Cannot read file at '%s'" % (filename,))
            return 1

    limbInterface = baxter_interface.Limb(limb)

    def getButtonPress(buttonpress):
        while not buttonpress.pressed:
            rospy.sleep(0.5)

        #Get points from user
        jointdict = limbInterface.joint_angles()
        print jointdict
        return [jointdict[limb+"_"+name] for name in traj.jointnames]
    
    common.send_image(filenames[0])
    #Get the current position
    buttonpress = common.ButtonListener()
    buttonpress.subscribe("/robot/digital_io/"+limb+"_lower_button/state")
    points = []
    points.append( getButtonPress(buttonpress))
    print "Got first position:"
    print points[0]

    buttonpress.pressed = False

    common.send_image(filenames[1])
    points.append( getButtonPress(buttonpress))
    print "Got second position:"
    print points[1]

    common.send_image(filenames[2])
    print("Running. Ctrl-c to quit")

    i = 0
    while i < 1000 and not rospy.is_shutdown():
        #Make sure the points alternate
        traj.add_point(points[i%2], 15.0)
        traj.add_point(points[(i+1)%2], 15.0)
        traj.start()
        traj.wait()
        traj.clear(limb)
        print "Completed test", i
        i+=1

if __name__ == "__main__":
    main()
