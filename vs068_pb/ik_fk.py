import time
import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from vs068_pb.utils import get_length, elapsed_time, randomize, interval_generator, matrix_from_quat, \
    get_joint_limits, get_min_limits, get_max_limits, get_distance, get_joint_positions, invert, \
    multiply, all_between, get_sample_fn, get_custom_limits, set_joint_states, euler_from_quat, quat_from_euler, \
    uniform_generator, Pose, Point, Euler, get_modded_pose_generator, save_state, restore_state, drawJointAABB, \
        checkAllowedContacts, quick_load_bot
import vs068_pb.config as config
from vs068_pb.config import INF
import numpy.random as random
from itertools import product, combinations, count, cycle, islice
from math import degrees, radians

if config.IKFAST_AVAILABLE:
    import ikfast_vs068 as ikv
    print("IKFast Imported to ik_fk")


##########

# Examples : https://github.com/yijiangh/conrob_pybullet/tree/master/debug_examples
# https://github.com/caelan/pybullet-planning/blob/b89756d0aaa5f8015cd3a59f85efab30a0ab42a1/pybullet_tools/ikfast/pr2/ik.py
# TODO (swilcock0) : Test IK
    
# IK SAMPLING : See https://github.com/caelan/pybullet-planning/blob/b280cfc578479309a15a5e4023a1bacf29eb25ee/pybullet_tools/ikfast/ikfast.py#L147

# TODO (swilcock0) : Implement a distance measure and collision checking
# TODO (swilcock0) : Write an RRT* motion planner
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
        def Fn(position=[0,0,0], orientation=[0,0,0,0], cid=0, *args, **kwargs):
            return ikv.get_ik(position, matrix_from_quat(cid, orientation).tolist())
    else:
        def Fn(position=[0,0,0], orientation=[1,0,0,0], cid=0, botId=0):
            return [list(p.calculateInverseKinematics(botId, config.EEF_ID, position, orientation, physicsClientId=cid)[0:6])]

    return Fn

def get_difference_fn(body, joints):
    def fn(q2, q1):
        return ((value2 - value1) for value2, value1 in [q2, q1])
    return fn

#############

def get_relative_pose(body, link1, link2=0):
    world_from_link1 = get_link_pose(body, link1)
    world_from_link2 = get_link_pose(body, link2)
    link2_from_link1 = multiply(invert(world_from_link2), world_from_link1)
    return link2_from_link1

def get_base_from_ee(robot, tool_link, world_from_target):
    ee_link = config.info.ee_link
    tool_from_ee = get_relative_pose(robot, ee_link, tool_link)
    world_from_base = get_link_pose(robot, 0)
    return multiply(invert(world_from_base), world_from_target, tool_from_ee)

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
#    print("Indices ", joint_indices)
#    print("Config ", conf)
    for i in range(len(joint_indices)):
        if joint_indices[i] in config.info.free_joints:
            index = config.info.free_joints.index(joint_indices[i])
            limits = config.lower_lims[index], config.upper_lims[index]
        else:
            limits = get_joint_limits(cid, botId, joint_indices[i])
            
        if conf[i] < limits[0] or conf[i] > limits[1]:
            #print("Limit {} < {} < {} invalid".format(limits[0], conf[i], limits[1]))
            return True
            break
        
    return False
      
# def select_solution(botId, joints, solutions, nearby_conf=False, rand=False, **kwargs):
#     """select one joint configuration given a list of them """
#     if not solutions:
#         return None
#     if rand and not nearby_conf:
#         return random.choice(solutions)
#     if nearby_conf:
#         nearby_conf = get_joint_positions(botId, joints)
#         # TODO: sort by distance before collision checking
#         # TODO: search over neighborhood of sampled joints when nearby_conf != None
#         return min(solutions, key=lambda conf: get_distance(nearby_conf, conf, **kwargs))
#     else:
#         return random.choice(solutions)

def get_valid_ik(pose, cid=0, botId=0, validate=True, validate_collisions=True, get_one=False, **kwargs):
    #botId, cid = quick_load_bot()
    if config.DEBUG:
        print("")
        print("POSE : {}".format(pose))
    solutions = sample_ik(cid, botId, pose, **kwargs)
    if validate:
        solutions = prune_invalid_poses(solutions, cid, botId, get_one, **kwargs)

    return solutions

def sample_ik(cid, robot, pose, attempts=100, num_candidates=INF, epsilon=config.CART_TOL, angle=config.ANGL_TOL, time_limit=INF, **kwargs):     
    gen_pose = get_modded_pose_generator(pose, epsilon=epsilon, angle=angle, **kwargs)
    solutions = []
    ikfast = getIK_FN()

    start_time = time.time()
    if num_candidates != INF:
        attempts = 99999

    for i in range(int(attempts)):
        if time.time() - start_time > time_limit and time_limit != INF:
            if config.DEBUG:
                print("Time limit reached for IK at pose {}".format(pose))
            break

        pose_test = next(gen_pose)

        ik_result = ikfast(pose_test[0], pose_test[1])
        if ik_result:
            for conf in ik_result:
                solutions.append(conf)
            
            if num_candidates == INF or len(solutions) > num_candidates:
                break

    if config.DEBUG:
        print("{} solutions found in {} seconds.".format(len(solutions), (time.time() - start_time)))

    if len(solutions) == 0:
        return []
        
    return solutions

def prune_invalid_poses(poses, cid=0, botId=0, get_one=False, validate_collisions=True, **kwargs):
    valid = []
    for i in range(len(poses)):
        # TODO (swilcock0) : Does this need checking??
        if violates_limits(cid, botId, config.info.free_joints, poses[i]) == False:
            valid.append(poses[i])
    if valid == []:
        if config.DEBUG:
            print("No valid poses found within joint limits")
        return []
    else:
        if validate_collisions:
            valid_coll = []
            #state_init = save_state(cid)

            for i in range(len(valid)):
                if checkAllowedContacts(valid[i], cid, botId):
                    valid_coll.append(valid[i])
                    if get_one:
                        break
                
            if config.DEBUG:
                print("{} valid poses found within joint limits and allowed contacts.".format(len(valid_coll)))
        
            #restore_state(state_init, cid)
            return valid_coll
        else:
            if config.DEBUG:
                print("{} valid poses found within joint limits.".format(len(valid)))
            return valid

#################