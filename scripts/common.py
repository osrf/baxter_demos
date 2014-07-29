import rospy
import sys

from copy import copy

import cv, cv2
import cv_bridge
import numpy

from baxter_core_msgs.msg import DigitalIOState

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import (
    Image,
)

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv.LoadImage(path)
    msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

#now extra portable
def colorSegmentation(img, blur_radius, radius, open_radius, color):

    #Get color of point in image
    #print self.point
    #Grab the R, G, B channels as separate matrices
    #use cv2.threshold on each of them
    #AND the three images together
    bw = numpy.ones(img.shape[0:2], numpy.uint8)
    maxvals = [179, 255, 255]
    for i in range(3):
        minval = color[i] - radius
        maxval = color[i] + radius
        if radius > color[i]:
            minval = 0
        elif radius + color[i] > maxvals[i]:
            minval = color[i] - radius

        channel = img[:, :, i]
        retval, minthresh = cv2.threshold(channel, minval, 255, cv2.THRESH_BINARY)
        retval, maxthresh = cv2.threshold(channel, maxval, 255, cv2.THRESH_BINARY_INV)
        bw = cv2.bitwise_and(bw, minthresh)
        bw = cv2.bitwise_and(bw, maxthresh)
    bw *= 255
    
    if open_radius != 0:
        open_kernel = numpy.array([open_radius, open_radius])

        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, open_kernel, iterations = 2)
    return bw


class MouseListener():
    def __init__(self):
        self.done = False
        self.x_clicked = -1
        self.y_clicked = -1
    def onMouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_LBUTTONDOWN:
            print "Got mouse event:", x, ",", y
            self.x_clicked = x
            self.y_clicked = y
            self.done = True



class ButtonListener:
    def subscribe(self, topic):
        rospy.Subscriber(topic, DigitalIOState, self.button_callback)
        self.pressed = False

    def getButtonPress(self, limb, limbInterface, traj):
        #Get points from user
        jointdict = limbInterface.joint_angles()
        print jointdict
        return [jointdict[limb+"_"+name] for name in traj.jointnames]


    def button_callback(self, data):
        if data.state == 1:
            self.pressed = True

class Trajectory(object):
    def __init__(self, limb):
        self.waiting = False
        self.jointnames = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self.waiting = True
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        self.waiting = False

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            self.jointnames]


