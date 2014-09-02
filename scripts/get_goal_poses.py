#!/usr/bin/env python

"""Uses the 3D camera to 3D centroid and pose for all objects in the scene"""

import argparse
import rospy
import yaml
import baxter_interface

import cv, cv2, cv_bridge
import numpy
import tf
from math import atan2, pi, sqrt

import common
import ik_command

import image_geometry

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from baxter_demos.msg import BlobInfo, BlobInfoArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion


config_folder = rospy.get_param('object_tracker/config_folder')
with open(config_folder+'servo_to_object.yaml', 'r') as f:
    params = yaml.load(f)

def unmap(points):
    return [points.x, points.y]

class PoseCalculator():
    def __init__(self, limb):
        self.limb = limb
        self.goal_poses = []
        self.pc = None
        self.tf_listener = tf.TransformListener()
        self.camera_model = None

    def publish(self):
        self.handler_pub = rospy.Publisher("object_tracker/"+self.limb+"/goal_poses", PoseArray)
        self.pub_rate = rospy.Rate(params['rate'])

    def subscribe(self):
        topic = "object_tracker/blob_info"
        self.centroid_sub = rospy.Subscriber(topic, BlobInfoArray, self.centroid_callback)
        self.pc_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pc_callback)
        self.info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.info_callback)

    def info_callback(self, data):
        # Get a camera model object using image_geometry and the camera_info topic
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.info_sub.unregister() #Only subscribe once

    def centroid_callback(self, data):
        if self.pc is None:
            return
        self.goal_poses = []
        
        for blob in data.blobs:
            centroid = (blob.centroid.x, blob.centroid.y)
            axis = numpy.concatenate((numpy.array(unmap(blob.axis.points[0])),\
                                      numpy.array(unmap(blob.axis.points[1]))))
            pos = self.solve_goal_point(centroid)

            theta = self.calculate_angle(axis)
            quat = tf.transformations.quaternion_from_euler(-pi, 0, theta)

            self.goal_poses.append( Pose(position=Point(*pos), orientation=Quaternion(*quat)))

    def get_depth(self, x, y):
        gen = pc2.read_points(self.pc, field_names='z', skip_nans=False, uvs=[(x, y)]) #Questionable
        print gen
        return next(gen)

    def solve_goal_point(self, centroid):
        # Find the centroid in the point cloud
        x = int(centroid[0])
        y = int(centroid[1])
        depth = self.get_depth(x, y)
        print depth
        # Get pixel points in camera units
        v = self.camera_model.projectPixelTo3dRay((x, y))
        d_cam = numpy.concatenate( (depth*numpy.array(v), numpy.ones(1))).reshape((4, 1))

        # TODO: is this the right frame transform?
        self.tf_listener.waitForTransform('/base', '/camera_depth_optical_frame', rospy.Time(), rospy.Duration(4))
        (trans, rot) = self.tf_listener.lookupTransform('/base', '/camera_depth_optical_frame', rospy.Time())
        
        camera_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

        d_base = numpy.dot(camera_to_base, d_cam)
        d_base[2] -= params['object_height']/2.0
        return d_base[0:3]


    def calculate_angle(self, axis):
        axis = axis[2:4] - axis[0:2]
        theta1 = atan2(axis[1], axis[0])

        ortho = numpy.array((axis[1], -axis[0]))
        theta2 = atan2(ortho[1], ortho[0])

        if abs(theta2) < abs(theta1):
            return -theta2
        else:
            return -theta1

    def pc_callback(self, data):
        # Do some processing on the pointcloud?
        self.pc = data

def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=False, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )

    args = parser.parse_args(rospy.myargv()[1:])
    if args.limb is None:
        limb = 'right'
    else:
        limb = args.limb

    rospy.init_node("get_goal_poses")

    pose_calc = PoseCalculator(limb)
    pose_calc.subscribe()
    pose_calc.publish()
    rate = rospy.Rate(params['rate'])
    while not rospy.is_shutdown():
        if len(pose_calc.goal_poses) > 0:
            print pose_calc.goal_poses
            pose_msg = PoseArray()
            pose_msg.poses = pose_calc.goal_poses
            pose_calc.handler_pub.publish(pose_msg)
        rate.sleep()

if __name__ == "__main__":
    main()
