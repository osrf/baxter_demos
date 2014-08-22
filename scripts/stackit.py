#!/usr/bin/env python
# TODO: make adaptive to the poses

import argparse
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject
import geometry_msgs.msg
import baxter_interface
from baxter_interface import CHECK_VERSION
import yaml
from super_stacker import DepthCaller, incrementPoseMsgZ


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
    required.add_argument('-v', '--visualize', required=False,
        choices=['true', 'false'], help='visualize robot in rviz')

    args = parser.parse_args(rospy.myargv()[1:])
    print args
    if args.visualize is None or 'false' in args.visualize:
        visualize = False
    else:
        visualize = True
    limb = args.limb

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('stackit')

    rate = rospy.Rate(1)
    # Get the goal poses
    dc = DepthCaller(limb)
    while (not dc.done) and (not rospy.is_shutdown()):
        print "No poses found yet"
        rate.sleep()
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(limb+"_arm")
    group.allow_replanning(True)

    if visualize:
        display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
        rospy.sleep(5)
    else:
        display_trajectory_publisher = None


    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    #Calibrate gripper
    gripper_if = baxter_interface.Gripper(limb)
    if not gripper_if.calibrated():
        print "Calibrating gripper"
        gripper_if.calibrate()


    goal_pose_publisher = rospy.Publisher('object_finder/next_goal_pose', geometry_msgs.msg.Pose)
    
    stack_pose = dc.object_pose_msgs[len(dc.object_pose_msgs)-1]
    stack_pose = incrementPoseMsgZ(stack_pose, params['object_height'])
    dc.object_pose_msgs.pop(len(dc.object_pose_msgs)-1)
    i = 0
    for pose in dc.object_pose_msgs:
        print "setting target to pose"
        # Move to the next block
        goal_pose_publisher.publish(pose)
        group.clear_pose_targets()
        group.set_pose_target(pose)

        # Remove object from collision matrix

        plan = group.plan()

        # is there a better way of checking this?
        plan_found = len(plan.joint_trajectory.points) > 0
        if visualize and plan_found:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)
            print "============ Waiting while RVIZ displays plan1..."
            rospy.sleep(5)

        group.go(wait=True)

        if plan_found:
            gripper_if.close(block=True)

        # Move to the stacking position
        group.clear_pose_targets()
        group.set_pose_target(stack_pose)
        plan = group.plan()
        plan_found = len(plan.joint_trajectory.points) > 0

        group.go(wait=True)

        if plan_found:
            gripper_if.open(block=True)
        
        # Get the next stack pose
        stack_pose = incrementPoseMsgZ(pose, params['object_height'])
        i+=1

if __name__ == "__main__":
    main()
