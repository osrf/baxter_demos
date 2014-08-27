#!/usr/bin/env python

import argparse
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject, PlanningScene
from baxter_demos.msg import CollisionObjectArray
import geometry_msgs.msg
import baxter_interface
from baxter_interface import CHECK_VERSION
import yaml
from super_stacker import incrementPoseMsgZ

class ObjectManager:
    # Subscribe to the /object_tracker/collision_objects topic
    # Make any necessary modifications received from this object's owner
    # Publish collision_objects to planning_scene

    def callback(self, data):
        self.planning_scene = PlanningScene()
        collision_objects = data.objects
        for obj in collision_objects:
            if obj.id in self.id_operations:
                obj.operation = self.id_operations[obj.id]
            else:
                obj.operation = CollisionObject.ADD
                self.id_operations[obj.id] = obj.operation
            print "Sending object with id "+obj.id+" to planning scene for operation" + str(obj.operation)
            self.planning_scene.world.collision_objects.append(obj)
        self.publish()
        
    def publish(self):
        self.pub.publish(self.planning_scene)

    def __init__(self):
        self.sub = rospy.Subscriber("object_tracker/collision_objects",
                                     CollisionObjectArray, self.callback)
        self.pub = rospy.Publisher("planning_scene", PlanningScene)
        self.id_operations = {}
        self.planning_scene = None
        

config_folder = rospy.get_param('object_tracker/config_folder')

with open(config_folder+'servo_to_object.yaml', 'r') as f:
    params = yaml.load(f)

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

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('stackit')

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
    while obj_manager.planning_scene is None:
        rate.sleep()
    objects = obj_manager.planning_scene.world.collision_objects
    
    if len(objects) > 1:
        stack_obj = objects[len(objects)-1]
        stack_obj.primitive_poses[0] = incrementPoseMsgZ(stack_obj.primitive_poses[0], params['object_height']/2.0)
        objects.pop(len(objects)-1)
    elif len(objects) == 1:
        stack_obj = objects[0]

    #i = 0
    for obj in objects:
        
        pose = obj.primitive_poses[0]
        if obj.id in obj_manager.id_operations:
            obj_manager.id_operations[obj.id] = CollisionObject.REMOVE 
        obj_manager.publish()

        print "setting target to pose"
        # Move to the next block
        group.clear_pose_targets()
        group.set_pose_target(pose)

        # Remove object from collision matrix

        plan = group.plan()

        # is there a better way of checking this?
        plan_found = len(plan.joint_trajectory.points) > 0
        
        print "============ Waiting while RVIZ displays plan1..."
        rospy.sleep(3)

        group.go(wait=True)

        if plan_found:
            gripper_if.close(block=True)

        # Move to the stacking position
        group.clear_pose_targets()
        group.set_pose_target(stack_obj)
        plan = group.plan()
        plan_found = len(plan.joint_trajectory.points) > 0

        print "============ Waiting while RVIZ displays plan2..."
        rospy.sleep(3)

        group.go(wait=True)

        if plan_found:
            gripper_if.open(block=True)
        
        # Get the next stack pose
        stack_obj = incrementPoseMsgZ(pose, params['object_height'])

        if obj.id in obj_manager.id_operations:
            obj_manager.id_operations[obj.id] = CollisionObject.ADD
        obj_manager.publish()
        #i+=1

if __name__ == "__main__":
    main()
