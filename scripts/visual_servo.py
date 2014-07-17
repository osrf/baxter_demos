#!/usr/bin/env python

"""Visual Servo-er by Jackie Kay
This node listens for the centroid of the desired object from object_finder, converts to
robot coordinates, and commands the hand to into grasping range
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

from sensor_msgs.msg import (
    Image, CameraInfo, Range
)

from geometry_msgs.msg import(Point)

alpha = 0.0001
inc = 0.01

class VisualCommand():
    def __init__(self, iksvc, limb):
        #Do some stuff
        self.centroid = None
        self.ir_reading = None
        self.cam_matrix = None
        self.iksvc = iksvc
        self.limb = limb
        self.limb_iface = baxter_interface.Limb(limb)
        self.tf_listener = tf.TransformListener()
        self.goal_pos = numpy.array((320, 100))
        self.range_limit = 0.1
        #self.table_height = 0.7315 #in meters
        #self.cam_dims = (640, 400)

    def subscribe(self):
        topic = "object_tracker/"+self.limb+"/centroid"
        self.centroid_sub = rospy.Subscriber(topic, Point, self.centroid_callback)
        topic = "/robot/range/"+self.limb+"_hand_range/state"
        self.ir_sub = rospy.Subscriber(topic, Range, self.ir_callback)
        topic = "/cameras/"+self.limb+"_hand_camera/camera_info"
        self.info_sub = rospy.Subscriber(topic, CameraInfo, self.info_callback)

    def command_ik(self, direction):
        """Use the Rethink IK service to figure out a desired joint position"""
        end_pose = self.limb_iface.endpoint_pose()
        current_p = numpy.array(end_pose['position']+end_pose['orientation']) 
        direction = numpy.concatenate((direction, numpy.zeros(4)))
        desired_p = current_p + direction
        print "Desired endpoint:", desired_p
        ik_command.service_request(self.iksvc, desired_p, self.limb)

    def centroid_callback(self, data):
        self.centroid = numpy.array((data.x, data.y))
        #Servo towards the centroid
        if self.centroid[0] == -1 or self.centroid[1] == -1:
            print "Waiting on centroid from object_finder"
            return
        #direction = alpha*self.camera_to_robot(self.centroid)

        d = self.centroid - self.goal_pos
        (trans, rot) = self.tf_listener.lookupTransform('/'+self.limb+'_hand_camera', '/base', rospy.Time(0))
        R = tf.transformations.quaternion_matrix(rot)[:3, :3]
        d = numpy.concatenate( (d, numpy.zeros(1)) )
        print d
        
        direction = alpha*numpy.dot(R, d) 

        #Calculate z-translate based on 
        direction[2] = 0
        if self.limb_iface.endpoint_pose()['position'][2] > -0.08 and self.ir_reading > 0.1:
            direction[2] = -inc
        else:
            direction = numpy.zeros(3)
        print "Desired change:", direction
        self.command_ik(direction)
        
    def ir_callback(self, data):
        self.ir_reading = data.range

    def info_callback(self, data):
        self.cam_matrix = numpy.array(data.P).reshape(3, 4) #3x4 Projection matrix
        self.cam_matrix = self.cam_matrix[:, :3]
        print self.cam_matrix
        # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
        #  the rectified image is given by:
        #  [u v w]' = P * [X Y Z 1]'
        #         x = u / w
        #         y = v / w
        self.info_sub.unregister() #Only subscribe once: might modify this
        
    def camera_to_robot(self, campos):
        #Convert from the camera frame (pixels units) to robot frame (meters)
        #Returns X, Y, Z: estimated position of the end effector
        x, y = campos
        x-=self.cam_dims[0]/2
        y-=self.cam_dims[1]/2
        w = self.limb_iface.endpoint_pose()['position'][2] - self.table_height
        P_inv = numpy.linalg.inv(self.cam_matrix)
        
        p_pix = numpy.array([x*w, y*w, w]).reshape(3, 1)
        p_cam = numpy.dot(P_inv, p_pix)
        #This gives us the position of the point relative to the position of the camera
        #in meters
        #TODO: need to rotate based on end effector pose
        #for now this is a corny approximation
        orientation = self.limb_iface.endpoint_pose()['orientation']
        R = tf.transformations.quaternion_matrix(orientation)[:3, :3]
        print R
        p_robot = numpy.dot(R, p_cam)
        #p_robot = p_cam
        #p_robot[0], p_robot[1] = -p_robot[1], p_robot[0]
        return p_robot[0:2].flatten()
        

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

    iksvc, ns = ik_command.connect_service(limb)

    command = VisualCommand(iksvc, limb)
    command.subscribe()
    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    main()
