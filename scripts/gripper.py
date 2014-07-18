"""Grip the object, move somewhere, and let go of the object
This is pretty dumb
"""
import argparse
import rospy
import baxter_interface
import common


class GripMover():

    def __init__(self, limb):
        self.limb_if = baxter_interface.Limb(limb)
        self.gripper_if = baxter_interface.Gripper(limb)
        self.gripper_if.calibrate()
        self.traj = common.Trajectory(limb)

        names = ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
        names = [limb+'_'+name for name in names]

        #TODO: Replace this with topic published from get_input node
        jvals=[1.187301128466797, 1.942403170440674, 0.08206797205810547, -0.996704015789795, -0.6734175651123048, 1.0266166411193849, 0.4985437554931641]
        self.final_jcommand = dict(zip(names, jvals))

    def close(self):
        self.gripper_if.close()

    def open(self):
        self.gripper_if.open()

    def isGripping(self):
        return self.gripper_if.force > 5 #?!?!?

    def move(self):
        traj.addPoint(self.final_jcommand)
        traj.start()
        traj.wait()


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
    gripper.close()
    if gripper.isGripping():
        gripper.move()
        gripper.open()
    else:
        print "Oh no! I'm not gripping anything!"


if __name__ == "__main__":
    main()
