#!/usr/bin/env python
import argparse
import rospy
import baxter_interface
import yaml

import cv, cv2, cv_bridge
import numpy
import tf
from math import atan2, pi

import common
import ik_command

import image_geometry

from baxter_demos.msg import BlobInfo

from sensor_msgs.msg import Image, CameraInfo, Range

from geometry_msgs.msg import Pose, Point, Quaternion


config_folder = rospy.get_param('object_tracker/config_folder')

with open(config_folder+'servo_to_object.yaml', 'r') as f:
    params = yaml.load(f)

def unmap(points):
    return [points.x, points.y]

class DepthEstimator:
    def __init__(self, limb):
        self.tf_listener = tf.TransformListener()
        self.limb = limb
        self.limb_iface = baxter_interface.Limb(limb)

        self.ir_reading = None
        self.camera_model = None
        self.goal_pose = None
        self.done = False

        """node = "servo_to_object/"
        self.min_ir_depth = rospy.get_param(node+"min_ir_depth")
        self.object_height = rospy.get_param("/estimate_depth/object_height")
        self.inc = rospy.get_param(node+"servo_speed")
        self.camera_x = rospy.get_param(node+"camera_x")
        self.camera_y = rospy.get_param(node+"camera_y")"""

        self.inc = params['servo_speed']
        self.min_ir_depth = params['min_ir_depth']
        self.camera_x = params['camera_x'] 
        self.camera_y = params['camera_y'] 
        self.object_height = params['object_height']
        
    def publish(self, rate=100):
        self.handler_pub = rospy.Publisher("object_tracker/"+self.limb+"/goal_pose", Pose)
        self.pub_rate = rospy.Rate(rate)


    def subscribe(self):
        topic = "object_tracker/"+self.limb+"/centroid"
        self.centroid_sub = rospy.Subscriber(topic, BlobInfo, self.centroid_callback)
        topic = "/robot/range/"+self.limb+"_hand_range/state"
        self.ir_sub = rospy.Subscriber(topic, Range, self.ir_callback)
        topic = "/cameras/"+self.limb+"_hand_camera/camera_info"
        self.info_sub = rospy.Subscriber(topic, CameraInfo, self.info_callback)

    def centroid_callback(self, data):
        self.centroid = (data.centroid.x, data.centroid.y)

        if self.centroid is not (-1, -1) and self.camera_model is not None:
            #make this more readable
            self.axis = numpy.concatenate( (numpy.array( unmap(data.axis.points[0]) ), numpy.array( unmap(data.axis.points[1])) ) )

            pos = self.solve_goal_pose()
            if pos is None:
                return
            #quat = self.limb_iface.endpoint_pose()['orientation']
            #Calculate desired orientation
            theta = self.calculate_angle()
            quat = tf.transformations.quaternion_from_euler(-pi, 0, theta)

            self.goal_pose = Pose(position=Point(*pos), orientation=Quaternion(*quat))
            # unregister once published. this is risky design, what if the position changes while waiting to publish?
            if self.goal_pose is not None:
                #self.centroid_sub.unregister()
                self.done = True
            else:
                #We want to decrease Z to get a valid IR reading
                goal_pose = self.limb_iface.endpoint_pose()
                goal_pose["position"].z -= self.inc
                self.goal_pose = goal_pose
            
    def calculate_angle(self):
        axis = self.axis[2:4] - self.axis[0:2]
        theta1 = atan2(axis[1], axis[0])

        ortho = numpy.array((axis[1], -axis[0]))
        theta2 = atan2(ortho[1], ortho[0])

        if abs(theta2) < abs(theta1):
            return -theta2
        else:
            return -theta1

    def solve_goal_pose(self):
        if self.ir_reading is None:
            return None
        # Project centroid into 3D coordinates
        # TODO: rosparametrize
        center = (self.centroid[0] - self.camera_x/2, self.centroid[1] - self.camera_y/2) 
        vec = numpy.array( self.camera_model.projectPixelTo3dRay(center) )
        # Scale it by the IR reading
        d_cam = ( self.ir_reading - self.min_ir_depth - self.object_height ) * vec
        d_cam = numpy.concatenate((d_cam, numpy.ones(1)))
        print "Camera vector:", d_cam

        # Now transform into the world frame
        try:
            self.tf_listener.waitForTransform('/base', '/'+self.limb+'_hand_camera', rospy.Time(), rospy.Duration(4))
            (trans, rot) = self.tf_listener.lookupTransform('/base', '/'+self.limb+'_hand_camera', rospy.Time(0))

        except tf.ExtrapolationException:
            return None
        
        camera_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

        d_base = numpy.dot(camera_to_base, d_cam)
        return d_base[0:3]

    def ir_callback(self, data):
        self.ir_reading = data.range
        if self.ir_reading > 60:
            print "Invalid IR reading"
            self.ir_reading = 0.4

    def info_callback(self, data):
        # Get a camera model object using image_geometry and the camera_info topic
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.info_sub.unregister() #Only subscribe once


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

    rospy.init_node("estimate_depth")
    

    de = DepthEstimator(limb)
    de.subscribe()
    de.publish()
    print "subscribed"
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if de.goal_pose is not None:
            de.handler_pub.publish(de.goal_pose)
        rate.sleep()

if __name__ == "__main__":
    main()
