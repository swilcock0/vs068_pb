import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R

# Error handling detects if the ikfast module is loaded locally - requires building from github.com/swilcock0/ikfast_pybind with VS2015 C++ build tools
try:
    import ikfast_vs068 as ikv
    IKFAST_AVAILABLE = True
except ModuleNotFoundError:
    IKFAST_AVAILABLE = False

EEF_ID = 8

def set_IKFAST(onoff=True):
    global IKFAST_AVAILABLE
    IKFAST_AVAILABLE=onoff
    if IKFAST_AVAILABLE:
        try:
            import ikfast_vs068 as ikv
            IKFAST_AVAILABLE = True
        except ModuleNotFoundError:
            IKFAST_AVAILABLE = False

# TODO (swilcock0) : Write and test FK function
# TODO (swilcock0) : Modify ik generator fn
# TODO (swilcock0) : Write an IK sampler function to get a closest fit
# TODO (swilcock0) : Write an RRT* motion planner
# TODO (swilcock0) : Implement collision detection. See caelan/pybullet_planning for example
# TODO (swilcock0) : Comments

def getFK_FN():
    ''' Get the forwards kinematics function '''
    global IKFAST_AVAILABLE
    if IKFAST_AVAILABLE:
        def Fn(joint_angles, *args, **kwargs):
            # TODO (swilcock0) : Test
            return ikv.get_fk(joint_angles)[0], R.from_matrix(ikv.get_fk(joint_angles)[1]).as_quat()
    else:
        def Fn(joint_angles=[0,0,0], botId=0, cid=0):
            joint_indices = []
            for i in range(p.getNumJoints(botId)):
                joint = p.getJointInfo(botId, i)
                if joint[2] == p.JOINT_REVOLUTE:
                    joint_indices.append(joint[0])
            
            for i in range(len(joint_indices)):
                p.resetJointState(botId, joint_indices[i], joint_angles[i], physicsClientId=cid)
            
            posorn = p.getLinkState(botId, EEF_ID)[0:2]
            
            pos = np.array(posorn[0]).tolist()        
            orn = np.array(posorn[1]).tolist()

            return pos, orn

    return Fn

def getIK_FN():
    ''' Get the inverse kinematics function (depending on whether IKFast is built and available this may be analytical, req. VSC++ on Windows) '''
    if IKFAST_AVAILABLE:
        def Fn(position=[0,0,0], orientation=[0,0,0,0], *args, **kwargs):
            # TODO (swilcock0) : the ikfast takes orientation as a rotation matrix!
            print(ikv.get_ik(position, np.reshape(p.getMatrixFromQuaternion(orientation), [3,3]).tolist()))   
    else:
        def Fn(position=[0,0,0], orientation=[1,0,0,0], botId=0, cid=0):
            # TODO (swilcock0) : Add joint limits
            print(p.calculateInverseKinematics(botId, EEF_ID, position, orientation, physicsClientId=cid))

    return Fn

# def get_ik_generator(robot, tool_pose, track_limits=False, prev_free_list=[]):
#     # 
#     world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
#     base_from_tool = multiply(invert(world_from_base), tool_pose)
#     base_from_ik = multiply(base_from_tool, get_tool_from_ik(robot))
#     sampled_limits = get_ik_limits(robot, joint_from_name(robot, *TRACK_JOINT), track_limits)
#     while True:
#         if not prev_free_list:
#             sampled_values = [random.uniform(*sampled_limits)]
#         else:
#             sampled_values = prev_free_list
#         ik_joints = get_track_arm_joints(robot)
#         confs = compute_inverse_kinematics(get_ik, base_from_ik, sampled_values)
#         yield [q for q in confs if not violates_limits(robot, ik_joints, q)]

# def get_tool_from_ik(robot):
#     world_from_tool = get_link_pose(robot, link_from_name(robot, TOOL_FRAME))
#     world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAME))
#     # tool from the bare flange (6th axis)
#     return multiply(invert(world_from_tool), world_from_ik)


# def sample_tool_ik(robot, tool_pose, max_attempts=10, closest_only=False, get_all=False, prev_free_list=[], **kwargs):
#     generator = get_ik_generator(robot, tool_pose, prev_free_list=prev_free_list, **kwargs)
#     ik_joints = get_movable_joints(robot)
#     for _ in range(max_attempts):
#         try:
#             solutions = next(generator)
#             if closest_only and solutions:
#                 current_conf = get_joint_positions(robot, ik_joints)
#                 solutions = [min(solutions, key=lambda conf: get_distance(current_conf, conf))]
#             solutions = list(filter(lambda conf: not violates_limits(robot, ik_joints, conf), solutions))
#             return solutions if get_all else select_solution(robot, ik_joints, solutions, **kwargs)
#         except StopIteration:
#             break
#     return None

#################