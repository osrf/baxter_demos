#!/usr/bin/env python
# Use MoveIt to plan a stacking of the object

import argparse
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from baxter_interface import CHECK_VERSION
import yaml
from super_stacker import DepthCaller, incrementPoseZ


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
    rospy.init_node('stackit', anonymous=True)

    rate = rospy.Rate(params['rate'])
    # Get the goal poses
    dc = DepthCaller(limb)
    while (not dc.done) and (not rospy.is_shutdown()):
        rate.sleep()

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(limb+"_arm")

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


    stack_pose = dc.object_poses[-1]
    stack_pose = incrementPoseZ(stack_pose, dc.object_height)
    dc.object_poses.pop(len(dc.object_poses)-1)

    for pose in dc.object_poses[:len(dc.object_poses)]:
        # Move to the next block
        group.set_pose_target(pose)
        plan = group.plan()
        if visualize:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)
            print "============ Waiting while RVIZ displays plan1..."
            rospy.sleep(5)

        group.go(wait=True)
        group.clear_pose_targets()
        gripper_if.close(block=True)

        # Move to the stacking position
        group.set_pose_target(stack_pose)
        plan = group.plan()
        group.go(wait=True)
        group.clear_pose_targets()

        gripper_if.open(block=True)
        
        # Get the next stack pose
        stack_pose = incrementPoseZ(pose, dc.object_height)

if __name__ == "__main__":
    main()
