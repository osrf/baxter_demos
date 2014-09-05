#!/usr/bin/env python

import argparse
import numpy
import rospy
import tf
import yaml


parser = argparse.ArgumentParser()
parser.add_argument('-r', '--rate', required=False, help='Transform broadcast rate')
parser.add_argument('-p', '--path', required=True, help='Path to transform yaml file')
args = parser.parse_args(rospy.myargv()[1:])
if args.rate is None:
    args.rate = 100

with open(args.path, 'r') as f:
    params = yaml.load(f)
print params

rospy.init_node('yaml_transform_publisher')

rate = rospy.Rate(float(args.rate))

tf_broadcaster = tf.TransformBroadcaster()
trans = numpy.array(params['trans'])
rot = numpy.array(params['rot'])
child = params['child']
parent = params['parent']

while not rospy.is_shutdown():
    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), child, parent)
    rate.sleep()
