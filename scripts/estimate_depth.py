#!/usr/bin/env python

import argparse
import rospy
import baxter_interface
import yaml

import cv, cv2, cv_bridge
import numpy
import tf
from math import atan2, pi, sqrt

import common
import ik_command

import image_geometry

from baxter_demos.msg import BlobInfo, BlobInfoArray

from sensor_msgs.msg import Image, CameraInfo, Range

from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

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

        self.ir_reading = 0.4
        self.camera_model = None
        self.goal_poses = []
        self.done = False

        self.inc = params['servo_speed']
        self.min_ir_depth = params['min_ir_depth']
        self.camera_x = params['camera_x'] 
        self.camera_y = params['camera_y'] 
        self.object_height = params['object_height']
        
    def publish(self, rate=100):
        self.handler_pub = rospy.Publisher("/object_tracker/"+self.limb+"/goal_poses", PoseArray)
        self.pub_rate = rospy.Rate(rate)


    def subscribe(self):
        topic = "/object_tracker/blob_info"
        self.centroid_sub = rospy.Subscriber(topic, BlobInfoArray, self.centroid_callback)
        topic = "/robot/range/"+self.limb+"_hand_range/state"
        self.ir_sub = rospy.Subscriber(topic, Range, self.ir_callback)
        topic = "/cameras/"+self.limb+"_hand_camera/camera_info"
        self.info_sub = rospy.Subscriber(topic, CameraInfo, self.info_callback)

    def currentCentroidDistance(self, blob):
        return sqrt((self.centroid[0]-blob.centroid.x)**2 +
                    (self.centroid[1]-blob.centroid.y)**2)
    
    def centroid_callback(self, data):
        self.goal_poses = []

        for blob in data.blobs:
            centroid = (blob.centroid.x, blob.centroid.y)
            
            if blob.axis is None or self.camera_model is None:
                return
            axis = numpy.concatenate((numpy.array(unmap(blob.axis.points[0])),\
                                      numpy.array(unmap(blob.axis.points[1]))))

            pos = self.solve_goal_pose(centroid)

            #Calculate desired orientation
            theta = self.calculate_angle(axis)
            quat = tf.transformations.quaternion_from_euler(-pi, 0, theta)

            self.goal_poses.append( Pose(position=Point(*pos), orientation=Quaternion(*quat)))
        self.done = True
            
    def calculate_angle(self, axis):
        axis = axis[2:4] - axis[0:2]
        theta1 = atan2(axis[1], axis[0])

        ortho = numpy.array((axis[1], -axis[0]))
        theta2 = atan2(ortho[1], ortho[0])

        if abs(theta2) < abs(theta1):
            return -theta2
        else:
            return -theta1

    def solve_goal_pose(self, centroid):
        # Project centroid into 3D coordinates
        center = (centroid[0] - self.camera_x/2, centroid[1] - self.camera_y/2) 
        vec = numpy.array( self.camera_model.projectPixelTo3dRay(center) )
        # Scale it by the IR reading
        d_cam = ( self.ir_reading - self.min_ir_depth - self.object_height ) * vec
        d_cam = numpy.concatenate((d_cam, numpy.ones(1)))
        #print "Camera vector:", d_cam

        # Now transform into the world frame
        self.tf_listener.waitForTransform('/base', '/'+self.limb+'_hand_camera', rospy.Time(), rospy.Duration(4))
        (trans, rot) = self.tf_listener.lookupTransform('/base', '/'+self.limb+'_hand_camera', rospy.Time())
        
        camera_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

        d_base = numpy.dot(camera_to_base, d_cam)
        return d_base[0:3]

    def ir_callback(self, data):
        self.ir_reading = data.range
        if self.ir_reading > 60:
            rospy.loginfo( "Invalid IR reading" )
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
        if len(de.goal_poses) > 0:
            print de.goal_poses
            pose_msg = PoseArray()
            pose_msg.poses = de.goal_poses
            de.handler_pub.publish(pose_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
