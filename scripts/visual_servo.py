#!/usr/bin/env python

"""Visual Servo-er by Jackie Kay
This node listens for the centroid of the desired object from object_finder, converts to
robot coordinates, and commands the hand to into grasping range
"""

# TODO:
# Get out of invalid joint position states

import sys
import argparse

from math import pi

import rospy
import baxter_interface
from urdf_parser_py.urdf import URDF

import cv, cv2, cv_bridge
import numpy
import tf

import common
import ik_command

from std_msgs.msg import Bool

from baxter_demos.msg import (
    BlobInfo
)

from sensor_msgs.msg import (
    Image, CameraInfo, Range
)

from geometry_msgs.msg import(Point)

class VisualCommand():
    def __init__(self, iksvc, limb):
        self.centroid = None
        #self.x_extremes = None
        self.axes = None
        self.ir_reading = None
        self.iksvc = iksvc
        self.limb = limb
        self.limb_iface = baxter_interface.Limb(limb)
        self.gripper_if = baxter_interface.Gripper(limb)
        self.tf_listener = tf.TransformListener()
        self.stateidx = 0
        self.states = self.wait_centroid, self.orient, self.servo_xy, self.servo_z, self.grip_state, self.done_state
        self.done = 0
        #robot_urdf = URDF.load_from_parameter_server()

        self.sign = 1

        #Check if we are exceeding the joint limit specified by robot_urdf
        robot = URDF.from_parameter_server()
        key = limb+"_w2"
        for joint in robot.joints:
            if joint.name == key:
                break
        
        self.wristlim = joint.limit

        paramnames = ["servo_speed", "min_pose_z", "min_ir_depth"]
        paramvals = []
        for param in paramnames:
            topic = "/servo_to_object/"
            paramvals.append(rospy.get_param(topic+param))
        self.inc, self.min_pose_z, self.min_ir_depth = tuple(paramvals)
        self.goal_pos = (rospy.get_param(topic+"camera_x")*float(rospy.get_param(topic+"goal_ratio_x")), rospy.get_param(topic+"camera_y")*float(rospy.get_param(topic+"goal_ratio_y")))
        self.grip_height = self.min_pose_z

    def publish(self, rate=100):
        self.handler_pub = rospy.Publisher("object_tracker/grasp_ready", Bool)
        self.pub_rate = rospy.Rate(rate)

    def subscribe(self):
        topic = "object_tracker/"+self.limb+"/centroid"
        self.centroid_sub = rospy.Subscriber(topic, BlobInfo, self.centroid_callback)
        topic = "/robot/range/"+self.limb+"_hand_range/state"
        self.ir_sub = rospy.Subscriber(topic, Range, self.ir_callback)

    def command_ik(self, direction):
        """Use the Rethink IK service to figure out a desired joint position"""
        end_pose = self.limb_iface.endpoint_pose()
        current_p = numpy.array(end_pose['position']+end_pose['orientation']) 
        direction = numpy.concatenate((direction, numpy.zeros(4)))
        desired_p = current_p + direction
        ik_command.service_request(self.iksvc, desired_p, self.limb)

    def command_ik_pose(self, direction):
        end_pose = self.limb_iface.endpoint_pose()
        current_p = numpy.array(end_pose['position']+end_pose['orientation']) 
        desired_p = current_p + direction
        ik_command.service_request(self.iksvc, desired_p, self.limb)

    def wait_centroid(self):
        pass

    def orient(self):
        print "Orienting"
        inc = pi/180
        #Turn until we are aligned with the axis provided by object_finder
        #TODO: more clever and efficient turning

        joint_name = self.limb + "_w2"
        joint_angle = self.limb_iface.joint_angle(joint_name)
        print joint_angle

        if joint_angle > self.wristlim.upper:
            self.sign = -1
        elif joint_angle < self.wristlim.lower:
            self.sign = 1

        joint_angle += self.sign*inc

        self.limb_iface.set_joint_positions(dict(zip([joint_name],[joint_angle])))


    def disoriented(self):
        #self.axis is neither parallel nor perpendicular to the camera x-axis
        x_axis = numpy.array([1, 0])
        #axis = numpy.array( [self.axis[2]-self.axis[0], self.axis[3] - self.axis[1]] )
        axis = self.axis[2:4] - self.axis[0:2]
        ctheta = numpy.dot(x_axis, axis/numpy.linalg.norm(axis))
        print "cos(theta) =", ctheta
        thresh = 0.1
        #Want to either be codirectional or orthogonal
        print "Is the axis orthogonal to the camera?", (abs(ctheta) < thresh)
        print "Is the axis codirectional with the camera?", (abs(ctheta-1) < thresh)
        return (abs(ctheta) > thresh) and (abs(ctheta-1) > thresh)

    def servo_xy(self):
        print "translating in XY at speed:", self.inc
        d = self.centroid - self.goal_pos

        self.tf_listener.waitForTransform('/'+self.limb+'_hand_camera', '/base', rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = self.tf_listener.lookupTransform('/'+self.limb+'_hand_camera', '/base', rospy.Time(0))
        R = tf.transformations.quaternion_matrix(rot)[:3, :3]
        d = numpy.concatenate( (d, numpy.zeros(1)) )
        d_rot = numpy.dot(R, d) 
        direction = self.inc*d_rot / numpy.linalg.norm(d_rot)
        if not self.outOfRange():
            direction[2] = self.inc
        else:
            direction[2] = 0
        
        direction *= self.inc/ numpy.linalg.norm(direction)
        print direction

        self.command_ik(direction)

    def servo_z(self):
        print "translating in Z"
        #Calculate z-translate
        d = self.centroid - self.goal_pos

        direction = numpy.array([0, 0, -self.inc])
        self.command_ik(direction)

    def grip_state(self):
        self.gripper_if.close()
        if not self.gripper_if.gripping():
            print "oh no! I'm not gripping anything"
        else:
            self.done = 1 

    def done_state(self):
        self.done = 1
        print "Done"
        
    def outOfRange(self):
        return (self.limb_iface.endpoint_pose()['position'][2] >= self.min_pose_z) and (self.ir_reading >= self.min_ir_depth)

    def visual_servo(self):
        d = self.centroid - self.goal_pos
        if self.done:
            self.done_state()
            return

        #So this is not a proper state machine right now. In fact it's pretty messy.
        
        #Maybe experiment with making this proportional to Z-coordinate or contour size
        threshold = 10
        if self.disoriented() and self.ir_reading > 0.15:
            print "I am disoriented"
            self.stateidx = 1
        elif abs(d[0]) > threshold and abs(d[1]) > threshold:
            self.stateidx = 2
        elif self.outOfRange(): 
            self.stateidx = 3
        elif self.gripper_if.gripping():
            self.stateidx = 5
        else:
            self.stateidx = 4
        
        self.states[self.stateidx]()

    def centroid_callback(self, data):
        
        self.centroid = numpy.array((data.centroid.x, data.centroid.y))
        print "centroid:", self.centroid

        if self.centroid[0] == -1 or self.centroid[1] == -1:
            print "Waiting on centroid from object_finder"
            self.stateidx = 0
            return
        print data.axis.points[0]

        def unmap(points):
            return [points.x, points.y]
        self.axis = numpy.concatenate( (numpy.array( unmap(data.axis.points[0]) ), numpy.array( unmap(data.axis.points[1])) ) )

        print self.axis
        self.visual_servo()

    def ir_callback(self, data):
        self.ir_reading = data.range

        

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

    rospy.init_node("visual_servo")

    #Bit of cheating: move the robot's arms to a neutral position before running

    dominant_joints=[1.187301128466797, 1.942403170440674, 0.08206797205810547, -0.996704015789795, -0.6734175651123048, 1.0266166411193849, 0.4985437554931641]

    off_joints = [-1.1255584018249511, 1.4522963092712404, 0.6354515406555176, -0.8843399232055664, 0.6327670742797852, 1.2751215284729005, -0.4084223843078614, ]
    

    names = ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
    off_limb = "right"
    if limb == "right": off_limb = "left"
    off_names = [off_limb+"_"+name for name in names]
    
    jointlists = {}
    jointlists[off_limb] = off_joints
    jointlists[limb] = dominant_joints

    for side in [limb, off_limb]:
        limb_if = baxter_interface.Limb(side)
        limb_names = [side+"_"+name for name in names]
        joint_commands = dict(zip(limb_names, jointlists[side] ))
        limb_if.move_to_joint_positions(joint_commands)

    #Calibrate gripper
    gripper_if = baxter_interface.Gripper(limb)
    if not gripper_if.calibrated():
        print "Calibrating gripper"
        gripper_if.calibrate()
    
    iksvc, ns = ik_command.connect_service(limb)

    command = VisualCommand(iksvc, limb)
    command.subscribe()
    command.publish()
    while not rospy.is_shutdown():
        command.handler_pub.publish(command.done)
        command.pub_rate.sleep()


if __name__ == "__main__":
    main()
