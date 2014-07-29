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

from geometry_msgs.msg import Pose, Point

#going to write something similar for point cloud data (publishes to same topic. might be bad practice?)

#TODO: use axis info from .object_tracker/centroid to command a pose

inc = 0.05

class DepthEstimator:
    def __init__(self, limb):
        self.tf_listener = tf.TransformListener()
        self.limb = limb
        self.limb_iface = baxter_interface.Limb(limb)

        self.ir_reading = None
        self.camera_model = None
        self.goal_pose = None

        self.min_ir_depth = rospy.get_param("/visual_servo/min_ir_depth")
        
    def publish(self, rate=100):
        #TODO: Estimate orientation as well as position
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
            pos = self.solve_goal_pose()
            if pos is None:
                return
            # Keep the same orientation
            quat = self.limb_iface.endpoint_pose()['orientation']
            self.goal_pose = Pose(position=Point(*pos), orientation=quat)
            # unregister once published. this is risky design, what if the position changes while waiting to publish?
            if self.goal_pose is not None:
                self.centroid_sub.unregister()
            else:
                #We want to decrease Z to get a valid IR reading
                goal_pose = self.limb_iface.endpoint_pose()
                goal_pose["position"].z -= inc
                self.goal_pose = goal_pose
            

    def solve_goal_pose(self):
        if self.ir_reading is None:
            return None
        # Project centroid into 3D coordinates
        # TODO: less hardcoding
        center = (self.centroid[0] - 320, self.centroid[1] - 200) 
        vec = numpy.array( self.camera_model.projectPixelTo3dRay(center) )
        # Scale it by the IR reading
        d_cam = ( self.ir_reading - self.min_ir_depth ) * vec
        d_cam = numpy.concatenate((d_cam, numpy.ones(1)))
        print "Camera vector:", d_cam

        # Now transform into the world frame
        try:
            #self.tf_listener.waitForTransform('/'+self.limb+'_hand_camera', '/base', rospy.Time(), rospy.Duration(4))
            #(trans, rot) = self.tf_listener.lookupTransform('/'+self.limb+'_hand_camera', '/base', rospy.Time(0))

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

    while not rospy.is_shutdown():
        if de.goal_pose is not None:
            # Publish goal_pose once
            de.handler_pub.publish(de.goal_pose)
            return
        de.pub_rate.sleep()

if __name__ == "__main__":
    main()
