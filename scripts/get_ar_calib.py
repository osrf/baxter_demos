#!/usr/bin/env python

import rospy
import tf
import numpy
from math import pi

from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs.msg
#from ar_track_alvar import AlvarMarkers

# TODO: add averaging/filtering, auto-add to param server or urdf or file

config_folder = rospy.get_param('object_tracker/config_folder')
with open(config_folder+'ar_calib.yaml', 'r') as f:
    params = yaml.load(f)

class markerSubscriber():
    def __init__(self, markernum=2):
        self.sub = rospy.Subscriber("/visualization_marker", Marker, self.callback)
        self.markernum = markernum
        self.pose = None
    def callback(self, data):
        self.pose = data.pose

def getPoseFromMatrix(matrix):
    trans, quat = getTfFromMatrix(numpy.linalg.inv(matrix))
    return Pose(position=Point(*trans), orientation=Quaternion(*quat))

def getMatrixFromPose(pose):
    trans = (pose.position.x, pose.position.y, pose.position.z )
    rot = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    return tf.transformations.compose_matrix(translate = trans, angles = rot)

def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles)

def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))
    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)
    source_target = tf.transformations.compose_matrix(translate = trans,
                                                     angles = euler)
    #print "looked up transform from", source, "to", target, "-", source_target
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

#markernum = '2'
markernum = params['markernum']
measured_translation = params['measured_translation']
forearm_marker_rot = numpy.array(params['forearm_marker_rot']).reshape((3,3))
squaredims = tuple(params['squaredims'])
#measured_translation = [0.06991+0.01036+0.0225, -0.0569, -0.0055]
#forearm_marker_rot = numpy.array( [[-1, 0, 0], [0, 0, -1], [0, -1, 0]] )

rospy.init_node("get_ar_calib")

tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

marker_pub = rospy.Publisher("ar_calib_markers", MarkerArray)

rate = rospy.Rate(100)

forearm_marker_trans = numpy.dot(numpy.linalg.inv(forearm_marker_rot),
                               -numpy.array(measured_translation).reshape((3, 1))).flatten()
#forearm_marker_trans = numpy.array(measured_translation)
print forearm_marker_trans
# Calculate transform from forearm to marker
forearm_marker = tf.transformations.compose_matrix(
                translate = forearm_marker_trans,
                angles = tf.transformations.euler_from_matrix(forearm_marker_rot))
marker_sub = markerSubscriber()

# Publish transform and marker
while not rospy.is_shutdown():

    # base to forearm
    base_forearm = lookupTransform(tf_listener, '/right_lower_forearm', '/base')

    # Compose transforms
    # base to marker = forearm to marker * base to forearm
    base_marker = forearm_marker.dot(base_forearm)
    trans, rot = getTfFromMatrix(numpy.linalg.inv(base_marker))
    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "/ar_marker_2", "/base")
    marker_pose = getPoseFromMatrix(base_marker)

    # marker to camera
    marker_camera = lookupTransform(tf_listener, '/camera_link', '/ar_marker_'+markernum)

    # base to camera = marker to camera * base to marker
    base_camera = marker_camera.dot(base_marker)
    trans, rot = getTfFromMatrix(numpy.linalg.inv(base_camera))

    tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "/camera_link", "/base")


    camera_pose = getPoseFromMatrix(base_camera)
    
    marker_msg = create_marker("marker_pose", 44, Marker.CUBE, marker_pose, (0, 255, 0), squaredims )

    camera_msg = create_marker("camera_pose", 1337, Marker.ARROW, camera_pose, (0, 0, 255), (0.2, 0.01, 0.01))

    # Now refine the transform using least squares
    # Get the pose of the marker from ar_pose_marker in the camera frame
    msg = [marker_msg, camera_msg]

    marker_pub.publish(msg)
    rate.sleep()

print "Writing transform to yaml file"
# Write to yaml file
f = open(config_folder+"/base_camera_tf.yaml", 'w')
lines = ['base_camera_trans: ', 'base_camera_rot: ']
for elem in trans:
    lines[0] += str(elem) + ', '
for elem in rot:
    lines[1] += str(elem) + ', '
f.writelines(lines)
f.close()
