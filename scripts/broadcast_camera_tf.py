#!/usr/bin/env python

import rospy
import tf

node_name = "broadcast_camera_tf"
rospy.init_node(node_name)

br = tf.TransformBroadcaster()

# Load the transform from file
# Eventually this might be folded into get_ar_calib.py (?)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    #these params could change during runtime, thus are in the loop
    trans = rospy.get_param(node_name+"/base_camera_trans")
    rot = rospy.get_param(node_name+"/base_camera_rot")
    # sendTransform takes child then parent 
    br.sendTransform(trans, rot, rospy.Time.now(), "/camera_link", "/base")
    rate.sleep()
