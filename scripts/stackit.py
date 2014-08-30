#!/usr/bin/env python

import argparse
import sys
import copy
import rospy
import numpy
import cv2
import tf
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject, PlanningScene
from baxter_demos.msg import CollisionObjectArray
from baxter_demos.msg import BlobInfo, BlobInfoArray
import geometry_msgs.msg
from geometry_msgs.msg import(
    Point,
    Polygon, Quaternion, Pose
)
import baxter_interface
from baxter_interface import CHECK_VERSION
import yaml
from super_stacker import incrementPoseMsgZ
from object_finder import ObjectFinder
from visual_servo import VisualCommand
import ik_command

class ObjectManager:
    # Subscribe to the /object_tracker/collision_objects topic
    # Make any necessary modifications received from this object's owner
    # Publish collision_objects to planning_scene

    def callback(self, data):
        collision_objects = data.objects
        self.collision_objects = collision_objects

        for obj in collision_objects:
            self.id_operations[obj.id] = obj.operation

        """for obj in collision_objects:
            if obj.id in self.id_operations:
                obj.operation = self.id_operations[obj.id]
            else:
                obj.operation = CollisionObject.ADD
                self.id_operations[obj.id] = obj.operation
            print "Sending object with id "+obj.id+" to planning scene for operation"
                 + str(obj.operation)"""
        self.publish()
        
    def publish(self):
        for obj in self.collision_objects:
            obj.operation = self.id_operations[obj.id]
            self.pub.publish(obj)
            self.rate.sleep()


    def __init__(self):
        self.object_sub = rospy.Subscriber("object_tracker/collision_objects",
                                     CollisionObjectArray, self.callback)
        self.pub = rospy.Publisher("/collision_object", CollisionObject)
        self.id_operations = {}
        self.rate = rospy.Rate(100)
        self.collision_objects = []
        

config_folder = rospy.get_param('object_tracker/config_folder')

with open(config_folder+'servo_to_object.yaml', 'r') as f:
    params = yaml.load(f)

"""Project the given pose into a sensible position for the Baxter gripper
   Want the gripper to point downwards (z=(0, 0, -1))"""
# Violates orientation constraint for gripper
def projectPose(pose):
    pose.orientation = geometry_msgs.msg.Quaternion(0.6509160466, 0.758886809948,
                                 -0.0180992582839, -0.0084573527776)
    
    return pose
    """z = numpy.array([0, 0, -1]).reshape((3, 1))
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    R = tf.transformations.quaternion_matrix(q)
    # Check which column of the matrix is closest to Z
    col_i = min( [( numpy.linalg.norm( R[i, :3]- z), i) for i in range(3) ] )[1]
    u = R[(col_i+1) % 3, :3]
    v = R[(col_i-1) % 3, :3]
    u[2] = 0
    v[2] = 0
    u /= numpy.linalg.norm(u)
    v /= numpy.linalg.norm(v)
    # Orthogonalize with Gram-Schmidt
    proj_uv = u.dot(v)/u.dot(u) * u
    v = v - proj_uv
    v /= numpy.linalg.norm(v)

    # Now form a new quaternion and return it
    R2 = numpy.hstack( (z, u.reshape((3, 1)), v.reshape((3, 1)), numpy.zeros(3).reshape((3,1)) ) )
    R2 = numpy.vstack( (R2, numpy.array( (0, 0, 0, 1) )) )
    print R2
    pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_matrix(R2))
    return pose"""

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

    rospy.init_node('stackit')
    iksvc, ns = ik_command.connect_service(limb)

    moveit_commander.roscpp_initialize(sys.argv)

    rate = rospy.Rate(1)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(limb+"_arm")
    group.allow_replanning(True)

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    #Calibrate gripper
    gripper_if = baxter_interface.Gripper(limb)
    if not gripper_if.calibrated():
        print "Calibrating gripper"
        gripper_if.calibrate()

    obj_manager = ObjectManager()
    while len(obj_manager.collision_objects) <= 0:
        rate.sleep()
    objects = obj_manager.collision_objects
    
    """if len(objects) > 1:
        stack_pose = projectPose(objects[len(objects)-1].primitive_poses[0])
        stack_pose = incrementPoseMsgZ(stack_pose, params['object_height']*2.0)
        objects.pop(len(objects)-1)
    elif len(objects) == 1:
        stack_pose = projectPose(objects[0].primitive_poses[0])"""
    stack_pose = Pose(position=Point(0.593, -0.212, params['object_height']-0.130),
                      orientation=Quaternion(0.6509160466, 0.758886809948,
                                 -0.0180992582839, -0.0084573527776) )

    processed_win = "Processed image"
    raw_win = "Hand camera"
    cv2.namedWindow(processed_win)
    #cv2.namedWindow(raw_win)

    for obj in objects:
        print "Got pose:", obj.primitive_poses[0]
        pose = projectPose(obj.primitive_poses[0])
        pose = incrementPoseMsgZ(pose, params['object_height']*2)
        print "Modified pose:", pose
        if obj.id in obj_manager.id_operations:
            obj_manager.id_operations[obj.id] = CollisionObject.REMOVE 
        obj_manager.publish()

        print "setting target to pose"
        # Move to the next block
        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        group.set_pose_target(pose)

        plan = group.plan()

        # is there a better way of checking this?
        plan_found = len(plan.joint_trajectory.points) > 0

        if plan_found:
            #print "============ Waiting while RVIZ displays plan1..."
            #rospy.sleep(3)
            group.go(wait=True)

            imgproc = ObjectFinder("star", None, None)
            imgproc.subscribe("/cameras/"+limb+"_hand_camera/image")
            imgproc.publish(limb)

            vc = VisualCommand(iksvc, limb)
            vc.subscribe()

            while (not vc.done) and (not rospy.is_shutdown()):
                blobArray = []
                
                for centroid, axis in zip(imgproc.centroids, imgproc.axes):
                    blob = BlobInfo()
                    centroid = Point(*centroid)
                    blob.centroid = centroid
                    if axis is None:
                        axis = -1*numpy.ones(6)
                    blob.axis = Polygon([Point(*axis[0:3].tolist()),
                                         Point(*axis[3:6].tolist())])
                    blobArray.append(blob)

                msg = BlobInfoArray()
                msg.blobs = blobArray
                imgproc.handler_pub.publish(msg)
                if imgproc.processed is not None:
                    cv2.imshow(processed_win, imgproc.processed)
                cv2.waitKey(10)
            vc.unsubscribe()
            imgproc.unsubscribe()

            #gripper_if.close(block=True)
        else:
            print "Unable to plan path"
            # what to do?

        # Move to the stacking position
        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        group.set_pose_target(stack_pose)
        plan = group.plan()
        plan_found = len(plan.joint_trajectory.points) > 0

        if plan_found:
            #print "============ Waiting while RVIZ displays plan2..."
            #rospy.sleep(3)
            group.go(wait=True)
            gripper_if.open(block=True)
        
        # Get the next stack pose
        stack_pose = incrementPoseMsgZ(pose, params['object_height'])

        """if obj.id in obj_manager.id_operations:
            obj_manager.id_operations[obj.id] = CollisionObject.ADD

        obj_manager.publish()"""

if __name__ == "__main__":
    main()
