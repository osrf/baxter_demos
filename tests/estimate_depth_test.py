#!/usr/bin/env python

"""Test for depth estimation
TODO: Render Baxter's hand at the goal position in Rviz, as well as the arm position that induced this goal position
"""

import rospy

from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
import visualization_msgs.msg

global marker_est, marker_true
marker_est = None
marker_true = None

goal_position = Point( x=0.785, y=0, z=-0.166)
#play the rosbag (user)

def callback(data):
    global marker_est, marker_true
    #Publish it as a marker in rviz
    marker_est = Marker()
    marker_est.header.frame_id = "base"
    marker_est.ns = "est_pose"
    marker_est.id = 42
    marker_est.type = Marker.ARROW
    marker_est.action = Marker.ADD
    marker_est.pose = data
    marker_est.color.r, marker_est.color.g, marker_est.color.b = (0, 255, 0)
    marker_est.color.a = 1.0
    marker_est.scale.x, marker_est.scale.y, marker_est.scale.z = (0.2, 0.01, 0.01)
    
    #Also publish the hardcoded goal position as a marker in rviz
    marker_true = Marker()
    marker_true.header.frame_id = "base"
    marker_true.ns = "true_pose"
    marker_true.id = 43
    marker_true.type = Marker.CUBE
    marker_true.action = Marker.ADD
    marker_true.pose.position = goal_position
    marker_true.color.r, marker_true.color.g, marker_true.color.b = (255, 128, 0)
    marker_true.color.a = 1.0
    marker_true.scale.x, marker_true.scale.y, marker_true.scale.z = (0.1, 0.1, 0.1)
    
rospy.init_node("estimate_depth_test")

#Subscribe to object_tracker/right/goal_pose
goal_sub = rospy.Subscriber("/object_tracker/right/goal_pose", Pose, callback)
est_pub = rospy.Publisher("goal_estimate", Marker)
true_pub = rospy.Publisher("goal_true", Marker)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    if marker_est is not None:
        est_pub.publish(marker_est)
    if marker_true is not None:
        true_pub.publish(marker_true)
    rate.sleep()
