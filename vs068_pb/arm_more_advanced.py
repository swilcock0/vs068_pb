import pybullet as p
import time
import pybullet_data
from math import degrees, radians
#import numpy as np
#from numpy.linalg import inv
#from scipy.spatial.transform import Rotation as R

import vs068_pb.config as config
#import vs068_pb.ik_fk as ik_fk



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
    # TODO (swilcock0): Shift to a class structure and do away with global variables
    
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
    for key in config.params:
        if str(key).isalpha() == False: # Ensure the param is a slider (with a numeric key)
            control_array.append(key)
            position_array.append(radians(p.readUserDebugParameter(config.params[key], cid))) # Build the joint position array
    # Control the joints
    p.setJointMotorControlArray(botId, control_array, p.POSITION_CONTROL, position_array, positionGains=len(control_array)*[0.1])


####################
'''pyBullet convenience functions'''

# TODO (swilcock0) : Move to utils?

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
    # Turn off using config.params at top
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

#################
            
def Start():
    ''' Main program '''                           
    Disconnect() # Ensure any stray servers are shut down
    gui_options = "--width="+str(config.screen_width)+" --height="+str(config.screen_height) # Setup screen width/height string
    physicsClient = p.connect(p.GUI, options= gui_options) # Connect to a physics client 
        
    # Set engine config.params
    p.setPhysicsEngineParameter(fixedTimeStep = config.T_STEP, 
                                numSolverIterations = 50, 
                                numSubSteps = 4)
    p.setGravity(0,0,-9.81)

    # Set data folders
    p.setAdditionalSearchPath(config.src_fldr)
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
    botId = p.loadURDF(config.urdf, startPos, startOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
    
    # Add the parameter sliders etc
    config.params = SetupParams(botId, physicsClient)
    
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
