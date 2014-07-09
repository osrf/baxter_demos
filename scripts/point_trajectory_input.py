#!/usr/bin/env python

import argparse
import sys
import os
import threading

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

import baxter_interface.digital_io as DIO

import cv
import cv_bridge

from sensor_msgs.msg import (
    Image,
)

from baxter_common import DigitalIOState

from std_msgs.msg import Int8

class ButtonListener:
    def subscribe(self):
        rospy.Subscriber("/robot/digital_io/right_lower_button/state", , self.button_callback)
        self.pressed = False

    def button_callback(self, data):
        print "Got data in callback:", data
        if data == 1:
            self.pressed = True
        
def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv.LoadImage(path)
    msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


class Trajectory(object):
    def __init__(self, limb):
        self.jointnames = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            self.jointnames]


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
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)

    filenames = ["assets/getpoint1.png", "assets/getpoint2.png"]
    for filename in filenames:
        if not os.access(filename, os.R_OK):
            rospy.logerr("Cannot read file at '%s'" % (filename,))
            return 1

    def getButtonPress():
        buttonpress = ButtonListener()
        buttonpress.subscribe()
        
        while not buttonpress.pressed:
            rospy.sleep(1)

        #Get points from user
        jointdict = limb.joint_angles
        return [jointdict[name] for name in traj.jointnames]
    
    send_image(filenames[0])
    #Get the current position
    p1 = getButtonPress()
    print "Got first position:"
    print p1

    send_image(filenames[1])
    p2 = getButtonPress()

    print("Running. Ctrl-c to quit")

    i = 0
    while True:
        traj.add_point(p1, 7.0) #TODO: better time check
        traj.add_point(p2, 7.0)
        #traj.add_point([x * 0.75 for x in p1], 9.0)
        #traj.add_point([x * 1.25 for x in p1], 12.0)
        traj.start()
        traj.wait(15.0) #TODO: more sophisticated check for completion
        print "Completed test", i
        i+=1

if __name__ == "__main__":
    main()
