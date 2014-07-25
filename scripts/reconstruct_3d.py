#!/usr/bin/env python
import argparse
import rospy
import baxter_interface

import cv, cv2, cv_bridge
import numpy
import tf

import common
import ik_command

import image_geometry

from baxter_demos.msg import BlobInfo

from sensor_msgs.msg import Image, CameraInfo, Range

from geometry_msgs.msg import Point

class DepthEstimator:
    def __init__(self, limb):
        self.tf_listener = tf.TransformListener()
        self.limb = limb

        self.ir_reading = None
        #self.cam_matrix = None

        self.min_ir_depth = rospy.get_param("/visual_servo/min_ir_depth")

    def subscribe(self):
        topic = "object_tracker/"+self.limb+"/centroid"
        self.centroid_sub = rospy.Subscriber(topic, BlobInfo, self.centroid_callback)
        topic = "/robot/range/"+self.limb+"_hand_range/state"
        self.ir_sub = rospy.Subscriber(topic, Range, self.ir_callback)
        topic = "/cameras/"+self.limb+"_hand_camera/camera_info"
        self.info_sub = rospy.Subscriber(topic, CameraInfo, self.info_callback)

    def centroid_callback(self, data):
        self.centroid = (data.centroid.x, data.centroid.y)
        print self.solve_goal_pose()

    def solve_goal_pose(self):
        #TODO: use image_geometry instead
        #INCOMPLETE
        """(trans, rot) = self.tf_listener.lookupTransform('/'+self.limb+'_hand_camera', '/base', rospy.Time(0))
        print "Translation relative to base:", trans
        R = tf.transformations.quaternion_matrix(rot)[:3, :3]
        print "Rotation matrix relative to base:", R
        #Figure out goal pose based on self.centroid and height of the table
        d = numpy.concatenate( (self.centroid, numpy.array([self.ir_reading-self.min_ir_depth ])) )
        #Apply the camera matrix to convert d to meters
        d_scaled = numpy.dot( numpy.linalg.inv(self.cam_matrix), d)
        print "Scaled direction in camera frame:", d_scaled

        d_rot = numpy.dot(R, d_scaled)
        d_trans = d_rot + trans
        
        print "Final coordinate:", d_rot"""
        # Project centroid into 3D coordinates
        vec = numpy.array( self.camera_model.projectPixelTo3dRay(self.centroid) )
        # Scale it by the IR reading
        d_cam = ( self.ir_reading - self.min_ir_depth ) * vec
        d_cam = numpy.concatenate((d_cam, numpy.ones(1)))
        # Now transform into the world frame

        (trans, rot) = self.tf_listener.lookupTransform('/'+self.limb+'_hand_camera', '/base', rospy.Time(0))
        
        camera_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
        d_base = numpy.dot(camera_to_base, d_cam)
        return d_base[0:3]
        

    def ir_callback(self, data):
        self.ir_reading = data.range


    def info_callback(self, data):
        #self.cam_matrix = numpy.array(data.K).reshape(3, 3) #3x3 Projection matrix
        #print self.cam_matrix
        # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
        #  the rectified image is given by:
        #  [x y w]' = K * [X Y Z ]'
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

    rospy.init_node("reconstruct_3d")
    
    de = DepthEstimator(limb)
    de.subscribe()
    print "subscribed"

    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    main()
