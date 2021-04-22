import pybullet as p
import time
import pybullet_data
import os
import tkinter as tk
from math import degrees, radians
import numpy as np
#from numpy.linalg import inv
#from scipy.spatial.transform import Rotation as R

import vs068_pb.ik_fk

# Get screen dimensions
root = tk.Tk()
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Data folders and locations
src_fldr = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources")
urdf = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources", "vs068_with_gripper_pybullet.urdf")  

# Params
ENG_RATE = 30 # Hz
T_STEP = 1./ENG_RATE
CAMERA_RATE = 1 # Hz
MOTION_RATE = 0.5 # Hz
LINE_LIFE = 5 # s
prev_pose = [0,0,0]
a = 0
inc = 6/(ENG_RATE/MOTION_RATE)
DEBUG = True
CAMERA_ACTIVATED = True
EEF_ID=8 # Naughty
SIM_T = 0.0
PRINT_SIM_T = False

#################

''' Motion planning '''

# TODO (swilcock0) : Write and test FK function
# TODO (swilcock0) : Modify ik generator fn
# TODO (swilcock0) : Write an IK sampler function to get a closest fit
# TODO (swilcock0) : Write an RRT* motion planner
# TODO (swilcock0) : Implement collision detection. See caelan/pybullet_planning for example

def getFK_FN():
    ''' Get the forwards kinematics function '''
    if IKFAST_AVAILABLE:
        def Fn(joint_angles, *args, **kwargs):
            # TODO (swilcock0) : Test
            print(ikv.get_fk(joint_angles))  
    else:
        def Fn(joint_angles=[0,0,0], botId=0, cid=0):
            # TODO (swilcock0) : Move this to a class var
            joint_indices = []
            for joint in range(p.getNumJoints(botId)):
                if joint[2] == p.JOINT_REVOLUTE or joint[2] == p.JOINT_PRISMATIC:
                    joint_indices.append(joint[0])
            
            p.resetJointStatesMultiDof(botId, joint_indices, joint_angles, physicsClientId=cid)

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

    if DEBUG: 
        # Add text for EEF pose and line
        output_params['posetext'] = p.addUserDebugText(str(pose_rounded), p.getLinkState(botId, 11)[0], textSize=1, textColorRGB=[0,0,0], physicsClientId=cid)
    
    return output_params
            
def DrawEEFLine(botId, cid):
    ''' Draw the line at the EEF '''
    # TODO (swilcock0): Shift to a class structure and do away with global variables
    global prev_pose
    global params
    
    # Get current pose
    current_pose = p.getLinkState(botId, EEF_ID)[0]
    # Draw a line from the previous pose
    p.addUserDebugLine(lineFromXYZ=prev_pose, lineToXYZ=current_pose, physicsClientId=cid, lifeTime=LINE_LIFE, lineColorRGB=[1,0,0], lineWidth=2)
    
    # If it's changed significantly, update text
    # TODO (swilcock0) : Add a rounding function for tuples
    if tuple([round(x,2) if isinstance(x, float) else x for x in prev_pose]) != tuple([round(x,2) if isinstance(x, float) else x for x in current_pose]):
        pose_rounded = tuple([round(x,2) if isinstance(x, float) else x for x in current_pose])
        params['posetext'] = p.addUserDebugText(str(pose_rounded), p.getLinkState(botId, 11)[0], textSize=1, textColorRGB=[0,0,0], replaceItemUniqueId=params['posetext'], physicsClientId=cid)
    
    prev_pose = current_pose
  
def MiscControl(botId, cid):
    ''' Move some joints along a determined path '''
    global a
    global inc
    
    # Increment the motion counter
    a += inc
    
    # If the motion counter gets to 3 (nearly pi == 180*) start going the other way
    if abs(a) > 3:
        inc = -inc
        
    # Control the joints
    p.setJointMotorControlArray(botId, [2,4,5], p.POSITION_CONTROL, [a, a/4-1, a], positionGains=3*[0.1])
    
def ParamControl(botId, cid):
    ''' Move the joints based on slider values '''
    # Arrays to hold joint indices, and position values
    control_array = []
    position_array = []
    
    # Run through the parameters
    for key in params:
        if str(key).isalpha() == False: # Ensure the param is a slider (with a numeric key)
            control_array.append(key)
            position_array.append(radians(p.readUserDebugParameter(params[key], cid))) # Build the joint position array
    # Control the joints
    p.setJointMotorControlArray(botId, control_array, p.POSITION_CONTROL, position_array, positionGains=len(control_array)*[0.1])


####################
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
    global SIM_T
    controltype = 1
    buttonsave = p.readUserDebugParameter(params['button'], cid)
    camera_ctr = 0
            
    for i in range(0, steps):
        p.stepSimulation(cid)
        
        if sleep == 1:
            time.sleep(T_STEP)
        
        # Check for the button presses and flip the control type if so (1<->0)
        if p.readUserDebugParameter(params['button'], cid) > buttonsave:
            controltype = abs(controltype - 1)
            buttonsave = p.readUserDebugParameter(params['button'], cid)
             
        # Select the controller type based on button history
        if controltype == 0:
            ParamControl(botId, cid)
        else:
            MiscControl(botId, cid)
        
        camera_ctr += 1
        if camera_ctr == ENG_RATE/CAMERA_RATE:
            Camera(cid, botId, 11)
            camera_ctr = 0
        
        SIM_T += T_STEP
        if PRINT_SIM_T:
            print(SIM_T)
                
        # Add the line if DEBUG is True in params at top
        if DEBUG:
            DrawEEFLine(botId, cid)

def Camera(cid, botId, linkID, distance=0.2):
    ''' Rig a camera to a link '''
    # Turn off using params at top
    if not CAMERA_ACTIVATED:
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

#################
            
def Start():
    ''' Main program '''                        
    global params
    
    Disconnect() # Ensure any stray servers are shut down
    gui_options = "--width="+str(screen_width)+" --height="+str(screen_height) # Setup screen width/height string
    physicsClient = p.connect(p.GUI, options= gui_options) # Connect to a physics client 
        
    # Set engine params
    p.setPhysicsEngineParameter(fixedTimeStep = T_STEP, 
                                numSolverIterations = 50, 
                                numSubSteps = 4)
    p.setGravity(0,0,-9.81)

    # Set data folders
    p.setAdditionalSearchPath(src_fldr)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  
    
    # Some debug visualiser options
    #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
    
    # Move the camera to somewhere useful
    p.resetDebugVisualizerCamera( cameraDistance=1.2, 
                                    cameraYaw=20.0, 
                                    cameraPitch=-40.0, 
                                    cameraTargetPosition=[0,0,0.8])
    
    # Load a floor
    planeId = p.loadURDF("plane.urdf")
    
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    
    # Load the robot
    botId = p.loadURDF(urdf, startPos, startOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
    
    # Add the parameter sliders etc
    params = SetupParams(botId, physicsClient)
    
    # Change colour of base box
    #p.changeVisualShape(botId, 0, rgbaColor=[0.0, 0.0, 1, 0.5])
    
    # Print some joint info
    print("Joints")
    print("--------")
    for i in range(p.getNumJoints(botId)):
        print(p.getJointInfo(botId, i))
#        print(str(p.getJointInfo(botId, i)[0]) + " : " + p.getJointInfo(botId, i)[1].decode('UTF-8') +
#            "   (" + str(p.getJointInfo(botId, i)[8]) + " < x < " + str(p.getJointInfo(botId, i)[9]) + ")")
    

    # Start stepping the sim
    Step(cid=physicsClient, botId=botId)
        
    Disconnect()

#####################
#Start()
