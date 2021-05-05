import numpy as np
import time
import random
from math import degrees, radians
import math
import pybullet as p
import vs068_pb.config as config
from numpy import array, concatenate
import sys
import os
import pybullet_data

INF = config.INF
'''pyBullet convenience functions'''


class HideOutput(object):
    '''
    A context manager that block stdout for its scope, usage:

    with HideOutput():
        os.system('ls -l')
    '''
    DEFAULT_ENABLE = True
    def __init__(self, enable=None):
        if enable is None:
            enable = self.DEFAULT_ENABLE
        self.enable = enable
        if not self.enable:
            return
        sys.stdout.flush()
        self._origstdout = sys.stdout
        self._oldstdout_fno = os.dup(sys.stdout.fileno())
        self._devnull = os.open(os.devnull, os.O_WRONLY)

    def __enter__(self):
        if not self.enable:
            return
        self._newstdout = os.dup(1)
        os.dup2(self._devnull, 1)
        os.close(self._devnull)
        sys.stdout = os.fdopen(self._newstdout, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.enable:
            return
        sys.stdout.close()
        sys.stdout = self._origstdout
        sys.stdout.flush()
        os.dup2(self._oldstdout_fno, 1)
        os.close(self._oldstdout_fno) # Added

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler) # TODO: extrinsic (static) vs intrinsic (rotating)

def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat) # rotation around fixed axis

def Point(x=0., y=0., z=0.):
    """Representing a point in 3D
    Parameters
    ----------
    x : float, optional
        [description], by default 0.
    y : float, optional
        [description], by default 0.
    z : float, optional
        [description], by default 0.
    Returns
    -------
    np array of three floats
        [description]
    """
    return np.array([x, y, z])

def Euler(roll=0., pitch=0., yaw=0.):
    """Representing a 3D rotation by Eulerian angles
    .. image:: ../images/roll_pitch_yaw.png
        :scale: 60 %
        :align: center
    `image source <https://devforum.roblox.com/t/take-out-pitch-from-rotation-matrix-while-preserving-yaw-and-roll/95204>`_
    Parameters
    ----------
    roll : float, optional
        [description], by default 0.
    pitch : float, optional
        [description], by default 0.
    yaw : float, optional
        [description], by default 0.
    Returns
    -------
    np array of three floats
        [description]
    """
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    """Representing a pose (or frame) in 3D
    Parameters
    ----------
    point : np array of three-floats, optional
        [description], by default None
    euler : np array of three eulerian angles, optional
        (roll, pitch, yaw), by default None
    Returns
    -------
    tuple of point, quaternion
        [description]
    """
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return (point, quat_from_euler(euler))

def Disconnect():
    ''' Disconnect any and all instances of bullet '''
    with HideOutput():
        for i in range(100):
            try:
                p.disconnect(i)
            except:
                break # Don't whinge about non-existent servers

def Step(steps = 10000, sleep = 1, cid=0, botId=0):
    ''' Step simulation steps times. If sleep == 0, no wait between steps '''
    controltype = 1
    config.buttonsave = p.readUserDebugParameter(config.params['button'], cid)
    camera_ctr = 0
            
    for i in range(0, steps):
        p.stepSimulation(cid)
        
        if sleep == 1:
            time.sleep(config.T_STEP)
        
        # Check for the button presses and flip the control type if so (1<->0)
        if p.readUserDebugParameter(config.params['button'], cid) > config.buttonsave:
            controltype = abs(controltype - 1)
            config.buttonsave = p.readUserDebugParameter(config.params['button'], cid)
             
        # Select the controller type based on button history
        if controltype == 0:
            ParamControl(botId, cid)
        else:
            MiscControl(botId, cid)
        
        camera_ctr += 1
        if camera_ctr == config.ENG_RATE/config.CAMERA_RATE:
            Camera(cid, botId, 11)
            camera_ctr = 0
        
        config.SIM_T += config.T_STEP
        if config.PRINT_SIM_T:
            print(config.SIM_T)
                
        # Add the line if DEBUG is True in config.params at top
        if config.DEBUG:
            DrawEEFLine(botId, cid)
            


def Camera(cid, botId, linkID, distance=0.2):
    ''' Rig a camera to a link '''
    # Turn off using params at top
    if not config.CAMERA_ACTIVATED:
        return
    
    # Get joint pose
    pos, orn = p.getLinkState(botId, linkID)[0:2]

    # Extract vectors from the rotation transformation
    rot_transform = p.getMatrixFromQuaternion(orn)
    forward_vector = [rot_transform[0], rot_transform[3], rot_transform[6]]
    up_vector = [rot_transform[2], rot_transform[5], rot_transform[8]]

    # Get a camera direction
    camera_target = [
            pos[0] + forward_vector[0],
            pos[1] + forward_vector[1],
            pos[2] + forward_vector[2]]

    # Form viewmatrix and projection matrix
    viewMatrix = p.computeViewMatrix(
                        pos,
                        camera_target,
                        up_vector)

    projectionMatrix = p.computeProjectionMatrixFOV(
                        fov=45.0,
                        aspect=1.0,
                        nearVal=0.1,
                        farVal=6.1)

    # Call the camera
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                        width=224, 
                        height=224,
                        viewMatrix=viewMatrix,
                        projectionMatrix=projectionMatrix)


''' Some "beauty" functions '''
def SetupParams(botId, cid):
    ''' Setup the parameters (sliders and buttons) '''
    output_params = {}
    
    # Run through the joints
    for i in range(p.getNumJoints(botId)):
        item = p.getJointInfo(botId, i) # Get the joint info
        # Here, check that the joint is movable
        if item[2] == p.JOINT_REVOLUTE or item[2] == p.JOINT_PRISMATIC:
            # If we can move it, assign a slider to it
            output_params[item[0]] = p.addUserDebugParameter(item[1].decode('UTF-8'), degrees(item[8]), degrees(item[9]), 0.0, cid)
            
    # Add button to change control method
    output_params['button'] = p.addUserDebugParameter('Switch control', 1, 0, 0, cid)
    
    # Round the pose to 2s.f.
    pose_rounded = tuple([round(x,2) if isinstance(x, float) else x for x in p.getLinkState(botId, 8)[0]])

    if config.DEBUG: 
        # Add text for EEF pose and line
        output_params['posetext'] = p.addUserDebugText(str(pose_rounded), p.getLinkState(botId, 11)[0], textSize=1, textColorRGB=[0,0,0], physicsClientId=cid)
    
    return output_params
            
def DrawEEFLine(botId, cid):
    ''' Draw the line at the EEF '''   
    # Get current pose
    current_pose = p.getLinkState(botId, config.EEF_ID)[0]
    # Draw a line from the previous pose
    p.addUserDebugLine(lineFromXYZ=config.prev_pose, lineToXYZ=current_pose, physicsClientId=cid, lifeTime=config.LINE_LIFE, lineColorRGB=[1,0,0], lineWidth=2)
    
    # If it's changed significantly, update text
    # TODO (swilcock0) : Add a rounding function for tuples
    if tuple([round(x,2) if isinstance(x, float) else x for x in config.prev_pose]) != tuple([round(x,2) if isinstance(x, float) else x for x in current_pose]):
        pose_rounded = tuple([round(x,2) if isinstance(x, float) else x for x in current_pose])
        config.params['posetext'] = p.addUserDebugText(str(pose_rounded), p.getLinkState(botId, 11)[0], textSize=1, textColorRGB=[0,0,0], replaceItemUniqueId=config.params['posetext'], physicsClientId=cid)
    
    config.prev_pose = current_pose
  
def MiscControl(botId, cid):
    ''' Move some joints along a determined path '''    
    # Increment the motion counter
    config.a += config.inc
    
    # If the motion counter gets to 3 (nearly pi == 180*) start going the other way
    if abs(config.a) > 3:
        config.inc = -config.inc
        
    # Control the joints
    p.setJointMotorControlArray(botId, [2,4,5], p.POSITION_CONTROL, [config.a, config.a/4-1, config.a], positionGains=3*[0.1])
    
def ParamControl(botId, cid):
    ''' Move the joints based on slider values '''
    # Arrays to hold joint indices, and position values
    control_array = []
    position_array = []
    
    # Run through the parameters
    for key in config.params:
        if str(key).isalpha() == False: # Ensure the param is a slider (with a numeric key)
            control_array.append(key)
            position_array.append(radians(p.readUserDebugParameter(config.params[key], cid))) # Build the joint position array
    # Control the joints
    p.setJointMotorControlArray(botId, control_array, p.POSITION_CONTROL, position_array, positionGains=len(control_array)*[0.1])


''' Helper utils - nicked from caelan/pybullet_planning '''

#####################################

def save_state(cid):
    return p.saveState(physicsClientId=cid)

def restore_state(state_id, cid):
    p.restoreState(stateId=state_id, physicsClientId=cid)

def save_bullet(filename, cid):
    p.saveBullet(filename, physicsClientId=cid)

def restore_bullet(filename, cid):
    p.restoreState(fileName=filename, physicsClientId=cid)

#####################################

def matrix_from_quat(cid, quat):
    return np.array(p.getMatrixFromQuaternion(quat, physicsClientId=cid)).reshape(3, 3)

def check_same(a, b, tol):
    #print(a)
    #print(b)
    flt_a = concatenate(array(a, dtype=object).reshape(-1))
    flt_b = concatenate(array(b, dtype=object).reshape(-1))
    
    same = True
    
    for i in range(len(flt_a)):
        if (flt_a[i] > (flt_b[i] + tol)) or (flt_a[i] < (flt_b[i] - tol)):
            same = False
            break
    return same

def all_between(lower_limits, values, upper_limits):
    ########## FIX
#    print(lower_limits)
#    print(values)
    assert len(lower_limits) == len(values)
    assert len(values) == len(upper_limits)
    return np.less_equal(lower_limits, values).all() and \
           np.less_equal(values, upper_limits).all()


def invert(pose):
    (point, quat) = pose
    return p.invertTransform(point, quat)

def multiply(*poses):
    pose = poses[0]
    for next_pose in poses[1:]:
        pose = p.multiplyTransforms(pose[0], pose[1], *next_pose)
    return pose


def get_length(vec, norm=2):
    return np.linalg.norm(vec, ord=norm)

def get_difference(p1, p2):
    from vs068_pb.rrt import TreeNode
    assert len(p1) == len(p2)
    return np.array(p2) - np.array(p1)

def get_distance(p1, p2, **kwargs):
    return get_length(get_difference(p1, p2), **kwargs)

def get_pose_distance(pose1, pose2):
    pos1, quat1 = pose1
    pos2, quat2 = pose2
    pos_distance = get_distance(pos1, pos2)
    ori_distance = quat_angle_between(quat1, quat2)
    return pos_distance, ori_distance

def quat_angle_between(quat0, quat1):
    # #p.computeViewMatrixFromYawPitchRoll()
    # q0 = unit_vector(quat0[:4])
    # q1 = unit_vector(quat1[:4])
    # d = clip(np.dot(q0, q1), min_value=-1., max_value=+1.)
    # angle = math.acos(d)
    
    # TODO: angle_between
    delta = p.getDifferenceQuaternion(quat0, quat1)
    d = clip(delta[-1], min_value=-1., max_value=1.)
    angle = math.acos(d)
    return angle

def get_joint_limits(cid=0, botId=0, joint=0):
    # TODO: make a version for several joints?
    #print(botId, cid, joint)
    if isinstance(joint, int):
        joint_info = p.getJointInfo(botId, joint, physicsClientId=cid)
        return list([joint_info[8], joint_info[9]])
    else:
        limits = []
        for i in joint:
            joint_info = p.getJointInfo(botId, i, physicsClientId=cid)
            limits.append([joint_info[8], joint_info[9]])
        return limits

def get_min_limit(botId, joint):
    # TODO: rename to min_position
    return get_joint_limits(botId=botId, joint=joint)[0]

def get_min_limits(botId, joints):
    return [get_min_limit(botId=botId, joint=joint) for joint in joints]

def get_max_limit(botId, joint):
    return get_joint_limits(botId=botId, joint=joint)[1]

def get_max_limits(botId, joints):
    return [get_max_limit(botId=botId, joint=joint) for joint in joints]  

def get_joint_positions(botId, joint_indices, cid=0):
    positions = []
    for i in range(len(joint_indices)):
        positions.append(p.getJointStates(botId, joint_indices, cid)[0][0])
    return positions

def get_delta_pose_generator(epsilon=0.1, angle=np.pi/6):
    lower = [-epsilon]*3 + [-angle]*3
    upper = [epsilon]*3 + [angle]*3

    if epsilon == -1:
        lower[:3] = [-1.2, -1.2, 0]
        upper[:3] = [1.2, 1.2, 1.5]

    for [x, y, z, roll, pitch, yaw] in interval_generator(lower, upper): # halton?
        pose = Pose(point=[x,y,z], euler=Euler(roll=roll, pitch=pitch, yaw=yaw))
        yield pose

def sample_line(segment, step_size=2e-2):
    (q1, q2) = segment
    diff = get_delta(q1, q2)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        yield tuple(np.array(q1) + l * diff / dist)
    yield q2

def get_delta(q1, q2):
    return np.array(q2) - np.array(q1)

def get_modded_pose_generator(pose_base, epsilon=0.1, angle=np.pi/6):
    lower = [-epsilon]*3 + [-angle]*3
    upper = [epsilon]*3 + [angle]*3

    base_pnt = pose_base[0]
    base_euler = euler_from_quat(pose_base[1])
    
    for [x, y, z, roll, pitch, yaw] in interval_generator(lower, upper): # halton?
        pose = Pose(point=[base_pnt[0]+x,base_pnt[1]+y,base_pnt[2]+z], euler=Euler(roll=base_euler[0] + roll, pitch=base_euler[0] + pitch, yaw=base_euler[0] + yaw))
        yield pose

def convex_combination(x, y, w=0.5):
    return (1-w)*np.array(x) + w*np.array(y)

def uniform_generator(d):
    while True:
        yield np.random.uniform(size=d)

def diff_all(a,b):
    ''' Return b - a '''
    if isinstance(a, (int, float)):
        return b - a
    else:
        assert len(a) == len(b)
        return [b[i] - a[i] for i in range(len(a))]

def less_than_tol(tol, lst):
    ''' Returns true if all less than tolerance '''
    if isinstance(lst, (float, int, np.float)):
        return lst - tol < 0
    
    diff = diff_all(tol, lst)

    for i in range(len(lst)):
        if diff[i] > 0:
            return False
    return True

def halton_generator(d):
    import ghalton
    seed = random.randint(0, 1000)
    #sequencer = ghalton.Halton(d)
    sequencer = ghalton.GeneralizedHalton(d, seed)
    #sequencer.reset()
    while True:
        [weights] = sequencer.get(1)
        yield np.array(weights)

def unit_generator(d, use_halton=True):
    if use_halton:
        try:
            import ghalton
        except ImportError:
            print('ghalton is not installed (https://pypi.org/project/ghalton/)')
            use_halton = False
    return halton_generator(d) if use_halton else uniform_generator(d)

def interval_generator(lower, upper, **kwargs):
    assert len(lower) == len(upper)
    assert np.less_equal(lower, upper).all()
    if np.equal(lower, upper).all():
        return iter([lower])
    return (convex_combination(lower, upper, w=weights) for weights in unit_generator(d=len(lower), **kwargs))

def elapsed_time(start_time):
    return time.time() - start_time

def safe_zip(sequence1, sequence2): # TODO: *args
    sequence1, sequence2 = list(sequence1), list(sequence2)
    assert len(sequence1) == len(sequence2)
    return list(zip(sequence1, sequence2))

def get_pairs(sequence):
    # TODO: lazy version
    sequence = list(sequence)
    return safe_zip(sequence[:-1], sequence[1:])

def get_wrapped_pairs(sequence):
    # TODO: lazy version
    sequence = list(sequence)
    # zip(sequence, sequence[-1:] + sequence[:-1])
    return safe_zip(sequence, sequence[1:] + sequence[:1])

def clip(value, min_value=-INF, max_value=+INF):
    return min(max(min_value, value), max_value)

def randomize(iterable): # TODO: bisect
    sequence = list(iterable)
    random.shuffle(sequence)
    return sequence

def get_random_seed():
    return random.getstate()[1][0]

def get_numpy_seed():
    return np.random.get_state()[1][0]

def set_random_seed(seed):
    if seed is not None:
        random.seed(seed)

def wrap_numpy_seed(seed):
    return seed % (2**32)

def set_numpy_seed(seed):
    # These generators are different and independent
    if seed is not None:
        np.random.seed(wrap_numpy_seed(seed))
        #print('Seed:', seed)

def get_custom_limits(cid, body, joints, custom_limits={}):
    joint_limits = []
    for joint in joints:
        if joint in custom_limits:
            joint_limits.append(custom_limits[joint])
        else:
            joint_limits.append(get_joint_limits(cid, body, joint))
    return zip(*joint_limits)

def get_sample_fn(cid, body, joints, custom_limits={}):
    lower_limits, upper_limits = get_custom_limits(cid, body, joints, custom_limits)
    def fn():
        return tuple(np.random.uniform(lower_limits, upper_limits))
    return fn

def set_joint_state(cid, body, joint, position, velocity):
    p.resetJointState(body, joint, targetValue=position, targetVelocity=velocity, physicsClientId=cid)

def set_joint_position(cid, body, joint, value):
    # TODO: remove targetVelocity=0
    p.resetJointState(body, joint, targetValue=value, targetVelocity=0, physicsClientId=cid)

# def set_joint_velocity(body, joint, velocity):
#     p.resetJointState(body, joint, targetVelocity=velocity, physicsClientId=CLIENT) # TODO: targetValue required

def set_joint_states(cid, body, joints, positions, velocities):
    #print(joints, positions)
    try:
        assert len(joints) == len(positions) == len(velocities)
        for joint, position, velocity in zip(joints, positions, velocities):
            set_joint_state(cid, body, joint, position, velocity)

            #print(joints, positions)
    except:
        print("Error")

def set_joint_positions(cid, body, joints, values):
    for joint, value in safe_zip(joints, values):
        set_joint_position(cid, body, joint, value)


def quick_load_bot(mode=p.DIRECT, physicsClient=-1, collisions = True):
    with HideOutput():
        if physicsClient == -1:
            physicsClient = p.connect(mode)

        p.resetDebugVisualizerCamera(cameraDistance=1.7, 
                                    cameraYaw=45.0, 
                                    cameraPitch=-20.0, 
                                    cameraTargetPosition=[0,0,0.8])

        p.setAdditionalSearchPath(config.src_fldr)
        startPos = [0,0,0]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        if collisions:
            botId = p.loadURDF(config.urdf, startPos, startOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
        else:    
                        botId = p.loadURDF(config.urdf, startPos, startOrientation, useFixedBase=1, flags=p.URDF_IGNORE_COLLISION_SHAPES)

        return botId, physicsClient

def irange(start, stop=None, step=1):  # np.arange
    if stop is None:
        stop = start
        start = 0
    while start < stop:
        yield start
        start += step


def negate(test):
    return lambda *args, **kwargs: not test(*args, **kwargs)


def argmin(function, sequence):
    # TODO: use min
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))], min(scores)

botId, cid = quick_load_bot()
config.LINK_IDS = {}
for i in range(p.getNumJoints(botId)):
        jointInfo = (p.getJointInfo(botId, i))

        config.LINK_IDS[jointInfo[1].decode("UTF-8")] = jointInfo[0]
p.disconnect(cid)


def getLinkFromName(name):
    return config.LINK_IDS.get(name, None)


config.NEVER_COLLIDE_NUMS = []
for pair in config.NEVER_COLLIDE_NAMES:
    config.NEVER_COLLIDE_NUMS.append([getLinkFromName(pair[0]), getLinkFromName(pair[1])])
#config.NEVER_COLLIDE_NUMS.append([0, -1])

#print(config.NEVER_COLLIDE_NAMES)
#print(config.NEVER_COLLIDE_NUMS)


def drawAABB(aabbMin, aabbMax):
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 0, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [0, 1, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [0, 0, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

def drawJointAABB(joint, cid=0, botId=0):
    aabb = p.getAABB(botId, joint, cid)
    
    drawAABB(aabb[0], aabb[1])

def checkAllowedContacts(conf, cid=0, botId=0):
    set_joint_states(cid, botId, config.info.free_joints, conf, [0]*6)
    p.stepSimulation(cid)

    allowedContacts = True

    for contact in (p.getContactPoints(physicsClientId=cid)):
        if ([contact[3], contact[4]] not in config.NEVER_COLLIDE_NUMS):
            allowedContacts = False
            
            if config.DEBUG:
                print(contact)

    return allowedContacts

def loadFloor(cid=0):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf", physicsClientId=cid)

def create_box_collisions(dims, pos, safety=1.2):
    ''' 
    Creates box collision and visual functions

    Example usage:
    dims = [[0.2, 0.2, 0.2], [0.1, 0.1, 1.2], [0.1, 0.1, 1.2]]
    pos = [[0.3,0.3,0.7], [-0.25, -0.25, 0.6], [-0.25, 0.25, 0.6]]
    collision_fn, visual_fn = create_box_collisions(dims, pos, safety=1.2)
    Gives a 20% safety boundary on sides

    Remember to Disconnect after using or their may be artifacts in display
    '''

    Disconnect()
    botId, cid = quick_load_bot(mode=p.DIRECT)
    
    allowed = []

    for i in range(len(dims)):
        box_lower = [pos[i][j] - dims[i][j]/2 for j in range(3)]
        box_upper = [pos[i][j] + dims[i][j]/2 for j in range(3)]
        box_halfs = [safety*abs(box_upper[i] - box_lower[i])/2 for i in range(3)]
        #print(box_halfs)
        box_centre = [box_lower[i] + box_halfs[i] for i in range(3)] 

        #print(box_lower, box_centre, box_upper, box_halfs)

        box_col = p.createCollisionShape(p.GEOM_BOX, 
                            physicsClientId=cid, 
                            halfExtents = box_halfs, 
                            )

        test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col, basePosition = box_centre, physicsClientId=cid)
        #print("Box : ID {} ".format(test_body))
    
    set_joint_states(cid, botId, config.info.free_joints, [0]*6, [0]*6)
    p.stepSimulation(cid)

    for contact in (p.getContactPoints(physicsClientId=cid)):
        if (([contact[3], contact[4]] not in config.NEVER_COLLIDE_NUMS) and contact[1] != contact[2]):                
            allowed.append(contact[:5])


    def check_contacts_against_init(a, allowed=allowed):
        #print(allowed)
        if contact[:5] in allowed:
            # print(contact[:5])
            # print(allowed)
            # input()
            return False
        else:
            return True


    def collision_fn(q):
        #print("Collision check")
        #print("Allowed : ".format(allowed))
        set_joint_states(cid, botId, config.info.free_joints, q, [0]*6)
        p.stepSimulation(cid)
        allowedContacts = True

        for contact in (p.getContactPoints(physicsClientId=cid)):
            if (([contact[3], contact[4]] not in config.NEVER_COLLIDE_NUMS) and contact[1] != contact[2]) or check_contacts_against_init(contact):                
                allowedContacts = False
                #if config.DEBUG:
                #print("Blocked : {}".format(contact[:5]))

        return not(allowedContacts)



    def create_visual_fn(cid):
        for i in range(len(dims)):
            box_lower = [pos[i][j] - dims[i][j]/2 for j in range(3)]
            box_upper = [pos[i][j] + dims[i][j]/2 for j in range(3)]
            box_halfs = [abs(box_upper[i] - box_lower[i])/2 for i in range(3)]
            box_centre = [box_lower[i] + box_halfs[i] for i in range(3)] 

            #print(box_lower, box_centre, box_upper, box_halfs)

            box_vis = p.createVisualShape(p.GEOM_BOX, 
                                rgbaColor=[1, 0.0, 0.0, 1], 
                                physicsClientId=cid, 
                                halfExtents = box_halfs, 
                                )
            test_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=box_vis, basePosition = box_centre, physicsClientId=cid)

    return collision_fn, create_visual_fn

def get_dist_fn(tool_space=False):
    from vs068_pb.ik_fk import getFK_FN
    from vs068_pb.rrt import TreeNode

    fk = getFK_FN()

    if tool_space:
        def fn(node_a, node_b=-1):
            if isinstance(node_b, TreeNode):
                #print("Node")
                test_pose = node_b.pose
            elif isinstance(node_b, int):
                #print("int")
                test_pose = desired_fk
            elif isinstance(node_b, list):
                #print("list")
                test_pose = fk(node_b)
            elif isinstance(node_b, np.ndarray):
                #print("NP Array")
                test_pose = fk(node_b)
            else:
                #print(type(node_b))
                test_pose = fk(node_b)

            distance = get_pose_distance(node_a.pose, test_pose)
            return distance #2*distance[0] + distance[1]
    else:
        def fn(conf_a, conf_b):
            if isinstance(conf_a, TreeNode):
                conf_a = conf_a.config
            if isinstance(conf_b, TreeNode):
                conf_b = conf_b.config 
            return get_distance(conf_a, conf_b)
            #return max([abs(conf_a[q] - conf_b[q]) for q in range(len(conf_a))])
    return fn

def size_all():
    import sys

    size_total = 0
    _ = dir()

    for key in _:
        size_total += eval('sys.getsizeof(' + key + ')')

    return size_total