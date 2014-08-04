# !/usr/bin/env python

import rospy
import tf
import numpy
from math import pi

from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs.msg

# TODO: add averaging/filtering, auto-add to param server or urdf or file

markernum = '2'
measured_translation = [0.06991+0.01036+0.0225, -0.0569, -0.0055]
forearm_marker_rot = numpy.array( [[-1, 0, 0], [0, 0, -1], [0, -1, 0]] )


def getPoseFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(numpy.linalg.inv(matrix))
    quat = tf.transformations.quaternion_from_euler(*angles)
    return Pose(position=Point(*trans), orientation=Quaternion(*quat))

def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles)

def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))
    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)
    source_target = tf.transformations.compose_matrix(translate = trans,
                                                     angles = euler)
    return source_target

def create_marker(ns, id_num, shape_type, pose, color, scale):
    # Create rviz marker message
    marker = Marker()
    marker.header.frame_id = "/base"
    marker.ns = ns
    marker.id = id_num
    marker.type = shape_type
    marker.action = Marker.ADD
    marker.pose = pose
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    return marker


forearm_marker_trans = numpy.dot(forearm_marker_rot,
                               -numpy.array(measured_translation).reshape((3, 1)))

rospy.init_node("get_ar_calib")

tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

# Calculate transform from forearm to marker
forearm_marker = tf.transformations.compose_matrix(
                translate = forearm_marker_trans.flatten(),
                angles = tf.transformations.euler_from_matrix(forearm_marker_rot))

marker_pub = rospy.Publisher("ar_calib_markers", MarkerArray)

rate = rospy.Rate(100)
# Publish transform and marker
while not rospy.is_shutdown():

    base_forearm = lookupTransform(tf_listener, '/right_lower_forearm', '/base')

    marker_camera = lookupTransform(tf_listener, '/camera_link', '/ar_marker_'+markernum)

    # Compose transforms
    base_marker = numpy.dot(forearm_marker, base_forearm)
    marker_pose = getPoseFromMatrix(base_marker)

    base_camera = numpy.dot(marker_camera, base_marker)
    print "Transformation:", getTfFromMatrix(base_camera)
    camera_pose = getPoseFromMatrix(base_camera)


    marker_msg = create_marker("marker_pose", 44, Marker.CUBE, marker_pose, (0, 255, 0), (0.05, 0.05, 0.02) )

    camera_msg = create_marker("camera_pose", 1337, Marker.ARROW, camera_pose, (0, 0, 255), (0.2, 0.01, 0.01))

    msg = [marker_msg, camera_msg]
    marker_pub.publish(msg)
    rate.sleep()

    #Save camera transform
