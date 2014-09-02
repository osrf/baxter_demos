import rospy
import struct
import baxter_interface
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

def connect_service(side):
    ns = "ExternalTools/"+side+"/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    return iksvc, ns

def service_request_velocity(iksvc, vel_vec, side):
    """Move in the requested direction at a constant velocity vector"""
    #INCOMPLETE
    ns = "ExternalTools/"+side+"/PositionKinematicsNode/IKService"
    ikreq = SolvePositionIKRequest()
    limb = baxter_interface.Limb(side)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')


def service_request(iksvc, desired_p, side, blocking=True):

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    pose = Pose(position=Point(x=desired_p[0], y=desired_p[1], z=desired_p[2]),
                orientation = Quaternion(x=desired_p[3], y=desired_p[4], z=desired_p[5], w=desired_p[6]))
            
    return service_request_pose(iksvc, pose, side, blocking)

def service_request_pose(iksvc, raw_pose, side, blocking=True):
    
    ns = "ExternalTools/"+side+"/PositionKinematicsNode/IKService"
    ikreq = SolvePositionIKRequest()
    limb = baxter_interface.Limb(side)
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    pose_stamped = PoseStamped( header = hdr, pose = raw_pose )
    
    ikreq.pose_stamp.append(pose_stamped)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        if blocking:
            limb.move_to_joint_positions(limb_joints)
        else:
            limb.set_joint_positions(limb_joints, raw=False)
    else:
        #How to recover from this
        print "Invalid position requested"
        return
