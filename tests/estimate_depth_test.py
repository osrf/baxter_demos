#!/usr/bin/env python

"""Test for depth estimation
TODO: Render Baxter's hand at the goal position in Rviz, as well as the arm position that induced this goal position
"""

import rospy

from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs.msg

global marker_ests
marker_ests = MarkerArray()

goal_position = Point( x=0.785, y=0, z=-0.166)
#play the rosbag (user)

def callback(data):
    global marker_ests
    #Publish it as a marker in rviz
    marker_ests = MarkerArray()
    marker_ests.markers = []
    print len(data.poses)
    i = 0
    for pose in data.poses:
        marker_est = Marker()
        marker_est.header.frame_id = "base"
        marker_est.ns = "est_pose_"+str(i)
        marker_est.id = 42+i
        marker_est.type = Marker.CUBE
        marker_est.action = Marker.ADD
        marker_est.pose = pose
        marker_est.color.r, marker_est.color.g, marker_est.color.b = (0, 255, 0)
        marker_est.color.a = 0.5
        marker_est.scale.x, marker_est.scale.y, marker_est.scale.z = (0.06, 0.06, 0.06)
        marker_ests.markers.append(marker_est)
        i+=1
    
   
rospy.init_node("estimate_depth_test")

#Subscribe to object_tracker/right/goal_pose
goal_sub = rospy.Subscriber("/object_tracker/right/goal_poses", PoseArray, callback)
est_pub = rospy.Publisher("object_estimates", MarkerArray)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    est_pub.publish(marker_ests)
    rate.sleep()
