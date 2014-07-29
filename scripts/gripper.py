#!/usr/bin/env python
"""Grip the object, move somewhere, and let go of the object
This is pretty dumb at the moment
"""
import argparse
import rospy
import baxter_interface
import common

from std_msgs.msg import Bool

class GripMover():
    def __init__(self, limb):
        self.limb = limb
        self.limb_if = baxter_interface.Limb(limb)
        self.gripper_if = baxter_interface.Gripper(limb)
        if not self.gripper_if.calibrated():
            self.gripper_if.calibrate()
        self.gripper_if.open(block=True)
        self.traj = common.Trajectory(limb)
        #names = [limb+'_'+name for name in names]

        names = ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
        #TODO: Replace this with topic published from get_input node
        self.jvals=[1.187301128466797, 1.942403170440674, 0.08206797205810547, -0.996704015789795, -0.6734175651123048, 1.0266166411193849, 0.4985437554931641]
        self.final_jcommand = dict(zip(names, self.jvals))
        self.handler_sub = rospy.Subscriber("object_tracker/grasp_ready", Bool, self.callback)
        self.done = False

    def callback(self, data):
        if data.data==1:
            print "Picking up object"
            self.gripper_if.close()
            if self.gripper_if.gripping():
                #TODO: check continuously if we are holding something
                self.move()
            else:
                print "Oh no! I'm not gripping anything!"

    def isGripping(self):
        return self.gripper_if.force > 5 #?!?!?

    def move(self):
        jointdict = self.limb_if.joint_angles()
        curjoints = [jointdict[self.limb+"_"+name] for name in self.traj.jointnames]
        jcommands = [self.final_jcommand[name] for name in self.traj.jointnames]
        self.traj.add_point(jcommands, 8.0)
        self.traj.start()
        self.traj.wait()

        self.gripper_if.open()
        self.handler_sub.unregister()
        self.done = True


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

    rospy.init_node("gripper")

    gripper = GripMover(limb)

    #Wait on signal from visual_servo
    while not rospy.is_shutdown() and not gripper.done:
        pass


if __name__ == "__main__":
    main()
