import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from vs068_pb.utils import matrix_from_quat, get_joint_limits, get_distance, get_joint_positions, invert, multiply, all_between
import vs068_pb.config as config
from vs068_pb.config import Pose, INF
import numpy.random as random

if config.IKFAST_AVAILABLE:
    import ikfast_vs068 as ikv
    print("IKFast Imported to ik_fk")


##########

# Examples : https://github.com/yijiangh/conrob_pybullet/tree/master/debug_examples
# TODO (swilcock0) : Test IK
    
# IK SAMPLING : See https://github.com/caelan/pybullet-planning/blob/b280cfc578479309a15a5e4023a1bacf29eb25ee/pybullet_tools/ikfast/ikfast.py#L147

# TODO (swilcock0) : Fix IK generator?? Or IK?
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
        def Fn(position=[0,0,0], orientation=[0,0,0,0], sampled=[], cid=0, *args, **kwargs):
            # Swap out for sampled ik, see https://github.com/caelan/pybullet-planning/blob/b280cfc578479309a15a5e4023a1bacf29eb25ee/pybullet_tools/ikfast/ikfast.py#L147
            print(position, orientation, sampled)
            if sampled:
                return ikv.get_ik(position, matrix_from_quat(cid, orientation).tolist(), sampled)
            else:
               return ikv.get_ik(position, matrix_from_quat(cid, orientation).tolist())
    else:
        def Fn(position=[0,0,0], orientation=[1,0,0,0], sampled=[], cid=0, botId=0):
            #print(list(p.calculateInverseKinematics(botId, config.EEF_ID, position, orientation, physicsClientId=cid)[0:6]))
            return [list(p.calculateInverseKinematics(botId, config.EEF_ID, position, orientation, physicsClientId=cid)[0:6])]

    return Fn

#############
    
#def ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
#                              fixed_joints=[], max_attempts=INF, max_time=INF,
#                              norm=INF, max_distance=INF, **kwargs):
#    assert (max_attempts < INF) or (max_time < INF)
#    if max_distance is None:
#        max_distance = INF
#    #assert is_ik_compiled(ikfast_info)
#    ikfast = import_ikfast(ikfast_info)
#    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
#    free_joints = joints_from_names(robot, ikfast_info.free_joints)
#    base_from_ee = get_base_from_ee(robot, ikfast_info, tool_link, world_from_target)
#    difference_fn = get_difference_fn(robot, ik_joints)
#    current_conf = get_joint_positions(robot, ik_joints)
#    current_positions = get_joint_positions(robot, free_joints)
#
#    # TODO: handle circular joints
#    free_deltas = np.array([0. if joint in fixed_joints else max_distance for joint in free_joints])
#    lower_limits = np.maximum(get_min_limits(robot, free_joints), current_positions - free_deltas)
#    upper_limits = np.minimum(get_max_limits(robot, free_joints), current_positions + free_deltas)
#    generator = interval_generator(lower_limits, upper_limits)
#    if max_attempts < INF:
#        generator = islice(generator, max_attempts)
#    start_time = time.time()
#    for free_positions in generator:
#        if max_time < elapsed_time(start_time):
#            break
#        for conf in randomize(compute_inverse_kinematics(ikfast.get_ik, base_from_ee, free_positions)):
#            #solution(robot, ik_joints, conf, tool_link, world_from_target)
#            difference = difference_fn(current_conf, conf)
#            if not violates_limits(robot, ik_joints, conf) and (get_length(difference, norm=norm) <= max_distance):
#                #set_joint_positions(robot, ik_joints, conf)
#                yield conf
#
#
#def closest_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
#                               max_candidates=INF, norm=INF, verbose=True, **kwargs):
#    start_time = time.time()
#    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
#    current_conf = get_joint_positions(robot, ik_joints)
#    generator = ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, norm=norm, **kwargs)
#    if max_candidates < INF:
#        generator = islice(generator, max_candidates)
#    solutions = list(generator)
#    # TODO: relative to joint limits
#    difference_fn = get_difference_fn(robot, ik_joints) # get_distance_fn
#    #set_joint_positions(robot, ik_joints, closest_conf)
#    solutions = sorted(solutions, key=lambda q: get_length(difference_fn(q, current_conf), norm=norm))
#    if verbose:
#        min_distance = min([INF] + [get_length(difference_fn(q, current_conf), norm=norm) for q in solutions])
#        print('Identified {} IK solutions with minimum distance of {:.3f} in {:.3f} seconds'.format(
#            len(solutions), min_distance, elapsed_time(start_time)))
#    return iter(solutions)
#
##############


def compute_inverse_kinematics(ik_fn, pose, sampled=[]):
    """ Compute ik solutions using the given ik function handle """
    # TODO : Add botid, cid
    pos = pose[0]
    rot = pose[1]
    if sampled:
        # ADD CID AND BOTID
        solutions = ik_fn(list(pos), list(rot), sampled)
    else:
        solutions = ik_fn(list(pos), list(rot))
    if solutions is None:
        return []
    return solutions


def get_link_pose(botId, link):
#    print(botId)
#    print(link)
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
#    print("Indices ", joint_indices)
#    print("Config ", conf)
    for i in range(len(joint_indices)):
        limits = get_joint_limits(0, botId, joint_indices[i])
        
        if conf[i] < limits[0] or conf[i] > limits[1]:
            return True
            break
        
    return False
      

def select_solution(botId, joints, solutions, nearby_conf=False, rand=False, **kwargs):
    """select one joint configuration given a list of them """
    if not solutions:
        return None
    if rand and not nearby_conf:
        return random.choice(solutions)
    if nearby_conf:
        nearby_conf = get_joint_positions(botId, joints)
        # TODO: sort by distance before collision checking
        # TODO: search over neighborhood of sampled joints when nearby_conf != None
        return min(solutions, key=lambda conf: get_distance(nearby_conf, conf, **kwargs))
    else:
        return random.choice(solutions)

def get_ik_generator(botId, tool_pose):
    ik = getIK_FN()
    world_from_base = get_link_pose(botId, config.info.base_link)
    #world_from_base == get_pose(robot)
    base_from_tool = multiply(invert(world_from_base), tool_pose)
    base_from_ik = base_from_tool #multiply(base_from_tool, get_tool_from_ik(botId))
    yield compute_inverse_kinematics(ik, base_from_ik)

def sample_tool_ik(botId, tool_pose, closest_only=False, get_all=False, **kwargs):
    generator = get_ik_generator(botId, tool_pose)
    ik_joints = config.info.free_joints
    solutions = next(generator)
    if closest_only and solutions:
        current_conf = get_joint_positions(botId, ik_joints)
        solutions = [min(solutions, key=lambda conf: get_distance(current_conf, conf))]
    solutions = list(filter(lambda conf: not violates_limits(botId, ik_joints, conf), solutions))
    return solutions if get_all else select_solution(botId, ik_joints, solutions, **kwargs)
#################