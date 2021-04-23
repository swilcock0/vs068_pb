import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from vs068_pb.utils import get_joint_limits, get_distance, get_joint_positions, invert, multiply, all_between
import vs068_pb.config as config
from vs068_pb.config import Pose
import numpy.random as random

if config.IKFAST_AVAILABLE:
    import ikfast_vs068 as ikv
    print("IKFast Imported to ik_fk")


##########

# Examples : https://github.com/yijiangh/conrob_pybullet/tree/master/debug_examples
# TODO (swilcock0) : Test IK
# TODO (swilcock0) : Shift all Poses to namedtuple Pose
# TODO (swilcock0) : Test generator
# TODO (swilcock0) : Test and understand IK sampler function to get a closest fit
# TODO (swilcock0) : Write an RRT* motion planner
# TODO (swilcock0) : Implement collision detection. See caelan/pybullet_planning for example
# TODO (swilcock0) : Comments
# TODO (swilcock0) : Add link selection to FK and use it for get link pose
# TODO (swilcock0) : Add cid throughout

def getFK_FN():
    ''' Get the forwards kinematics function '''
    # Select the function
    if config.IKFAST_AVAILABLE:
        def Fn(joint_angles, *args, **kwargs):
            # Use the IKFAST solver for forward kinematics
            return ikv.get_fk(joint_angles)[0], R.from_matrix(ikv.get_fk(joint_angles)[1]).as_quat()
    else:
        def Fn(joint_angles=[0,0,0], botId=0, cid=0):
            # Use pyBullet for FK
            joint_indices = []
            for i in range(p.getNumJoints(botId)):
                joint = p.getJointInfo(botId, i)
                if joint[2] == p.JOINT_REVOLUTE:
                    joint_indices.append(joint[0])
            
            for i in range(len(joint_indices)):
                p.resetJointState(botId, joint_indices[i], joint_angles[i], physicsClientId=cid)
            
            pos, orn = get_link_pose(botId, config.EEF_ID)
            
            # Jiggery pokery to make the results match
            pos = np.array(pos).tolist()  
            orn = np.array(orn).tolist()  

            return pos, orn
    return Fn

def getIK_FN():
    ''' Get the inverse kinematics function (depending on whether IKFast is built and available this may be analytical, req. VSC++ on Windows) '''
    if config.IKFAST_AVAILABLE:
        def Fn(position=[0,0,0], orientation=[0,0,0,0], sampled=[], *args, **kwargs):
            #print(orientation)
            if sampled:
                #print(ikv.get_ik(position, np.reshape(p.getMatrixFromQuaternion(orientation), sampled, [3,3]).tolist()))   
                return [ikv.get_ik(position, np.reshape(p.getMatrixFromQuaternion(orientation), sampled, [3,3]).tolist())]
            else:
                #print(ikv.get_ik(position, np.reshape(p.getMatrixFromQuaternion(orientation), [3,3]).tolist()))   
                return [ikv.get_ik(position, np.reshape(p.getMatrixFromQuaternion(orientation), [3,3]).tolist())]
    else:
        def Fn(position=[0,0,0], orientation=[1,0,0,0], sampled=[], botId=0, cid=0):
            #print(list(p.calculateInverseKinematics(botId, config.EEF_ID, position, orientation, physicsClientId=cid)[0:6]))
            return [list(p.calculateInverseKinematics(botId, config.EEF_ID, position, orientation, physicsClientId=cid)[0:6])]

    return Fn

def compute_inverse_kinematics(ik_fn, pose, sampled=[]):
    """ Compute ik solutions using the given ik function handle """
    # TODO : Add botid, cid
    pos = pose[0]
    rot = pose[1]
    if sampled:
        solutions = ik_fn(list(pos), list(rot), sampled)
    else:
        solutions = ik_fn(list(pos), list(rot))
    if solutions is None:
        return []
    return solutions


def get_link_pose(botId, link):
    # TODO (swilcock0) : Better integration with IKFAST, see getFK_FN
    state = p.getLinkState(botId, link)
    pos = state[0]
    orn = state[1]
    
    return pos, orn

def get_tool_from_ik(botId=0):
    world_from_tool = get_link_pose(botId, config.EEF_ID)
    world_from_ik = get_link_pose(botId, config.EEF_ID-1)
    # tool from the bare flange (6th axis)
    return multiply(invert(world_from_tool), world_from_ik)

def violates_limits(cid, botId, joint_indices, conf):
    ''' Check if a config violates the joint limits '''
    print("Indices ", joint_indices)
    print("Config ", conf)
    for i in range(len(joint_indices)):
        limits = get_joint_limits(0, botId, joint_indices[i])
        
        if conf[i] < limits[0] or conf[i] > limits[1]:
            return True
            break
        
    return False
   
def get_ik_generator(ik_fn, robot, ik_pose):
    # TODO (swilcock0) : Test me first!!!
    ik = ik_fn
   
    world_from_base = get_link_pose(robot, config.info.base_link)
    base_from_ik = multiply(invert(world_from_base), ik_pose)
    sampled_joints = config.info.free_joints
    sampled_limits = get_joint_limits(robot, joint=sampled_joints)
    arm_joints = config.info.free_joints

    min_limits = []
    max_limits = []
    for i in sampled_limits:
        min_limits.append(i[0])
        max_limits.append(i[0])

    while True:
        sampled_values = [random.uniform(*limits) for limits in sampled_limits]
        confs = compute_inverse_kinematics(ik, base_from_ik, sampled_values)
        
        print("CONFS: ",confs)
        solutions = [q for q in confs if all_between(min_limits, q, max_limits)]
        # TODO: return just the closest solution
        #print(len(confs), len(solutions))
        yield solutions
        if all(lower == upper for lower, upper in sampled_limits):
            break
    

def select_solution(botId, joints, solutions, nearby_conf=True, random=False, **kwargs):
    """select one joint configuration given a list of them """
    if not solutions:
        return None
    if random and not nearby_conf:
        return random.choice(solutions)
    if nearby_conf:
        nearby_conf = get_joint_positions(botId, joints)
        # TODO: sort by distance before collision checking
        # TODO: search over neighborhood of sampled joints when nearby_conf != None
        return min(solutions, key=lambda conf: get_distance(nearby_conf, conf, **kwargs))
    else:
        return random.choice(solutions)

def sample_tool_ik(botId, tool_pose, nearby_conf=False, max_attempts=25, **kwargs):
    ik_pose = multiply(tool_pose, get_tool_from_ik(botId))
    
    ik_fn = getIK_FN()
    generator = get_ik_generator(ik_fn, botId, ik_pose, **kwargs)
    
    joint_indices = []
    for i in range(p.getNumJoints(botId)):
        joint = p.getJointInfo(botId, i)
        if joint[2] == p.JOINT_REVOLUTE:
            joint_indices.append(joint[0])
            
    for _ in range(max_attempts):
        try:
            solutions = next(generator)
            # TODO: sort by distance from the current solution when attempting?
            if solutions:
                return select_solution(botId, joint_indices, solutions, nearby_conf=nearby_conf)
        except StopIteration:
            break
    return None
#################