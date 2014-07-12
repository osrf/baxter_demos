#!/usr/bin/env python
"""Baxter End Effector Controller Demo by Jackie Kay (jackie@osrfoundation.org)
Based on the Baxter RSDK Joint Position Example

Command the (x, y, z) position of the end effector using the joystick. Uses
Jacobian kinematics to determine required joint angles. 

"""

# TODO: refactor to use ik_command.py

import argparse
import rospy
import struct
import tf

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from baxter_pykdl import baxter_kinematics

from operator import add

import numpy as np
from math import atan2, asin

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
# Convert the last 4 entries in q from quaternion form to Euler Angle form and copy into p
# Expect a 7x1 column vector and a preallocated 6x1 column vector (numpy)
# p and q are both pose vectors
def quaternion_to_euler(q, p):
    if q.shape != (7, 1) or p.shape != (6, 1):
        raise Exception("Got unexpected vector dimension in quaternion_to_euler")
    p[0:3] = q[0:3]
    p[3], p[4], p[5] = tf.transformations.euler_from_quaternion(q[3:7].flatten().tolist())

# Get key commands from the user and move the end effector
def map_joystick(joystick):
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    
    #abbreviations
    jhi = lambda s: joystick.stick_value(s) > 0
    jlo = lambda s: joystick.stick_value(s) < 0
    bdn = joystick.button_down
    bup = joystick.button_up

    #These are from PyKDL and are needed for the Jacobian
    left_kin = baxter_kinematics('left')
    right_kin = baxter_kinematics('right')

    #Connect with IK service
    right_ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    right_iksvc = rospy.ServiceProxy(right_ns, SolvePositionIK)
    left_ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    left_iksvc = rospy.ServiceProxy(left_ns, SolvePositionIK)


 
    def command_jacobian(side, direction):
        if side == 'left':
            limb = left
            kin = left_kin
        elif side == 'right':
            limb = right
            kin = right_kin
        else:
            raise Exception("Got wrong side in command_jacobian")

        # current is the current position of end effector
        # We need to reshape it from quaternion to Euler Angle
        current_q = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation']) 
        current_q = current_q.reshape((7, 1))
        current = np.zeros((6, 1))
        quaternion_to_euler(current_q, current)

        # We need to convert the direction from the world frame to the hand frame
        # Rotate direction by the inverse of the rotation matrix from the angular pose in current
        R = tf.transformations.quaternion_matrix(current_q[3:7].flatten().tolist())
        R_inv = tf.transformations.inverse_matrix(R)
        print R_inv[0:3, 0:3]
        direction = np.array(direction).reshape((6, 1))
        print direction[0:3]
        dir_rot = np.dot(R_inv[0:3, 0:3], direction[0:3])
        print dir_rot
        direction[0:3] = dir_rot

        # Goal is the goal position, found by adding the requested direction from the user
        goal = current + direction
        current_angles = np.array([limb.joint_angles()[name] for name in limb.joint_names()])

        # Get the Jacobian inverse and solve for the necessary change in joint angles
        jacobian_inv = kin.jacobian_transpose()
        delta = jacobian_inv*(goal-current)
        commands = current_angles.flatten() + delta.flatten()

        # Command the joints
        command_list = commands.tolist()
        joint_command = dict(zip(limb.joint_names(), command_list[0]))
        limb.set_joint_positions(joint_command)

    def command_ik(side, direction):
        """Use the Rethink IK service to figure out a desired joint position
           This is way too slow to use in realtime. Unless I figure out why the
           IK service takes minutes to respond, not going to use it."""
        if side == 'left':
            limb = left
            iksvc = left_iksvc
            ns = left_ns
        else:
            limb = right
            iksvc = right_iksvc
            ns = right_ns
        current_p = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation']) 
        direction = np.array(direction)
        desired_p = current_p + direction
        print desired_p
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = { side : PoseStamped(
                    header = hdr,
                    pose = Pose(position=Point(x=desired_p[0], y=desired_p[1], z=desired_p[2]),
                    orientation = Quaternion(x=desired_p[3], y=desired_p[4], z=desired_p[5], w=desired_p[6]))
                ) }

        ikreq.pose_stamp.append(pose[side])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return
        print "Got response"
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            limb.set_joint_positions(limb_joints)
        else:
            #How to recover from this
            return

    zeros = [0]*4
    inc = 0.1

    def print_help(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s: %s" % (str(test[1][0]), doc))


    bindings_list = []
    bindings = (
        ((bdn, ['rightTrigger']),
         (grip_left.close,  []), "left gripper close"),
        ((bup, ['rightTrigger']),
         (grip_left.open,   []), "left gripper open"),
        ((bdn, ['leftTrigger']),
         (grip_right.close, []), "right gripper close"),
        ((bup, ['leftTrigger']),
         (grip_right.open,  []), "right gripper open"),
        ((jlo, ['leftStickHorz']),
         (command_ik, ['right', [0, inc, 0]+zeros]), lambda: "right y inc "),
        ((jhi, ['leftStickHorz']),
         (command_ik, ['right', [0, -inc, 0]+zeros],), lambda: "right y dec "),
        ((jlo, ['leftStickVert']),
         (command_ik, ['right', [inc, 0, 0]+zeros]), lambda: "right x inc "),
        ((jhi, ['leftStickVert']),
         (command_ik, ['right', [-inc, 0, 0]+zeros]), lambda: "right x dec "),
        ((bdn, ['dPadUp']),
         (command_ik, ['right', [0, 0, inc]+zeros]), lambda: "right z inc "),
        ((bdn, ['dPadDown']),
         (command_ik, ['right', [0, 0, -inc]+zeros]), lambda: "right z dec "),
        ((jlo, ['rightStickHorz']),
         (command_ik, ['left', [0, inc, 0]+zeros]), lambda: "left y inc "),
        ((jhi, ['rightStickHorz']),
         (command_ik, ['left', [0, -inc, 0]+zeros]), lambda: "left y dec "),
        ((jlo, ['rightStickVert']),
         (command_ik, ['left', [inc, 0, 0]+zeros]), lambda: "left x inc "),
        ((jhi, ['rightStickVert']),         
         (command_ik, ['left', [-inc, 0, 0]+zeros]), lambda: "left x dec "),
        ((bdn, ['btnUp']),
         (command_ik, ['left', [0, 0, inc]+zeros]), lambda: "left z inc "),
        ((bdn, ['btnDown']),
         (command_ik, ['left', [0, 0, -inc]+zeros]), lambda: "left z dec "),
        ((bdn, ['rightBumper']),
         (grip_left.calibrate, []), "left calibrate"),
        ((bdn, ['leftBumper']),
         (grip_right.calibrate, []), "right calibrate"),
        ((bdn, ['function1']),
         (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']),
         (print_help, [bindings_list]), "help"),
        )
    bindings_list.append(bindings)

    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press Ctrl-C to stop. ")
    while not rospy.is_shutdown():
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)
        """if len(lcmd):
            left.set_joint_positions(lcmd)
            lcmd.clear()
        if len(rcmd):
            right.set_joint_positions(rcmd)
            rcmd.clear()"""
        rate.sleep()
    return False


def main():
    """RSDK Joint Position Example: Joystick Control

    Use a game controller to control the angular joint positions
    of Baxter's arms.

    Attach a game controller to your dev machine and run this
    example along with the ROS joy_node to control the position
    of each joint in Baxter's arms using the joysticks. Be sure to
    provide your *joystick* type to setup appropriate key mappings.

    Each stick axis maps to a joint angle; which joints are currently
    controlled can be incremented by using the mapped increment buttons.
    Ex:
      (x,y -> e0,e1) >>increment>> (x,y -> e1,e2)
    """
    epilog = """
See help inside the example with the "Start" button for controller
key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--joystick', required=True,
        choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    joystick = None
    if args.joystick == 'xbox':
        joystick = baxter_external_devices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = baxter_external_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = baxter_external_devices.joystick.PS3Controller()
    else:
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_joystick")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_joystick(joystick)
    print("Done.")

if __name__ == '__main__':
    main()

