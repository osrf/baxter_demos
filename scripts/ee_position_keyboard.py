"""Baxter End Effector Controller Demo by Jackie Kay
Based on the Baxter RSDK Joint Position Example

Command the (x, y, z) position of the end effector using the keyboard. Uses
Jacobian kinematics to determine required joint angles. 
"""

# TODO:
# figure out coordinate frame and rotational frame: convert increment from world frame to
# robot frame

import argparse
import rospy
import tf

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from baxter_pykdl import baxter_kinematics

from operator import add

import numpy as np
from math import atan2, asin

# Convert the last 4 entries in q from quaternion form to Euler Angle form and copy into p
# Expect a 7x1 column vector and a preallocated 6x1 column vector (numpy)
# p and q are both pose vectors
def quaternion_to_euler(q, p):
    if q.shape != (7, 1) or p.shape != (6, 1):
        raise Exception("Got unexpected vector dimension in quaternion_to_euler")
    p[0:3] = q[0:3]
    p[3], p[4], p[5] = tf.transformations.euler_from_quaternion(q[3:7].flatten().tolist())

# Get key commands from the user and move the end effector
def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)

    #These are from PyKDL and are needed for the Jacobian
    left_kin = baxter_kinematics('left')
    right_kin = baxter_kinematics('right')
 
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

    zeros = [0]*3
    inc = 0.07
    bindings = {
        'q': (command_jacobian, ['right', [inc, 0, 0]+zeros], "increase right x"),
        'a': (command_jacobian, ['right', [-inc, 0, 0]+zeros], "decrease right x"),
        'w': (command_jacobian, ['right', [0, inc, 0]+zeros], "increase right y"),
        's': (command_jacobian, ['right', [0, -inc, 0]+zeros], "decrease right y"),
        'e': (command_jacobian, ['right', [0, 0, inc]+zeros], "increase right z"),
        'd': (command_jacobian, ['right', [0, 0, -inc]+zeros], "decrease right z"),
        'u': (command_jacobian, ['left', [inc, 0, 0]+zeros],  "increase left x"),
        'j': (command_jacobian, ['left', [-inc, 0, 0]+zeros], "decrease left x"),
        'i': (command_jacobian, ['left', [0, inc, 0]+zeros],  "increase left y"),
        'k': (command_jacobian, ['left', [0, -inc, 0]+zeros], "decrease left y"),
        'o': (command_jacobian, ['left', [0, 0, inc]+zeros],  "increase left z"),
        'l': (command_jacobian, ['left', [0, 0, -inc]+zeros], "decrease left z"),
        'z': (grip_right.close, [], "right: gripper close"), 
        'x': (grip_right.open, [], "right: gripper open"),
        'c': (grip_right.calibrate, [], "right: gripper calibrate")
    }

    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "command_jacobian(left, [0.1, 0, 0])"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))



def main():
    """RSDK End Effector Position Example: Keyboard Control

    Use your dev machine's keyboard to control end effector positions.

    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled



    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()

