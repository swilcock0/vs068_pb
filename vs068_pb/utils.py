import numpy as np
import time
from math import degrees, radians
import pybullet as p
import vs068_pb.config as config

'''pyBullet convenience functions'''

def Disconnect():
    ''' Disconnect any and all instances of bullet '''
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
                        farVal=3.1)

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
    assert len(p1) == len(p2)
    return np.array(p2) - np.array(p1)

def get_distance(p1, p2, **kwargs):
    return get_length(get_difference(p1, p2), **kwargs)

def get_joint_limits(cid=0, botId=0, joint=0):
    # TODO: make a version for several joints?
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
    return get_joint_limits(botId, joint)[0]

def get_min_limits(botId, joints):
    return [get_min_limit(botId, joint) for joint in joints]

def get_max_limit(botId, joint):
    return get_joint_limits(botId, joint)[1]

def get_max_limits(botId, joints):
    return [get_max_limit(botId, joint) for joint in joints]  

def get_joint_positions(botId, joint_indices, cid=0):
    positions = []
    for i in range(len(joint_indices)):
        positions.append(p.getJointStates(botId, joint_indices, cid)[0])
    return positions

def quick_load_bot():
    physicsClient = p.connect(p.GUI)
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    botId = p.loadURDF(config.urdf, startPos, startOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
    return botId, physicsClient