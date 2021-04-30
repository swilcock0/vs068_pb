import pybullet as p
import pybullet_data
import os
import time

# Setting some variables
src_fldr = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources")
urdf = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources", "vs068_with_gripper_pybullet.urdf")  

T_STEP = 1./100 # Timestep for physics


def Disconnect():
    ''' Disconnect any and all instances of bullet '''
    for i in range(100):
        try:
            p.disconnect(i)
        except:
            break # Don't whinge about non-existent servers

def Step(steps = 10000, sleep = 1, physicsClientID=0, botID=0):
        ''' Step simulation steps times. If sleep == 0, no wait between steps '''
        # Set a variable for the joint values and an increment
        a = 0
        inc = 1./100
        
        # Run x amount of steps
        for i in range(0, steps):
            p.stepSimulation(physicsClientID)
            
            if sleep == 1:
                time.sleep(T_STEP)
                
            # Move the joints a bit ([a, a/4, a] radians)
            a += inc
            if abs(a) > 3:
                inc = -inc
            p.setJointMotorControlArray(botID, [2,3,4,5], p.POSITION_CONTROL, [a, a/2, a/4-1, a])

def Start():
    ''' Run a demo '''
    
    
    Disconnect() # Just in case a server is already running
    physicsClientID = p.connect(p.GUI)  # Connect  

    # Add the resource folders to pyBullets path    
    p.setAdditionalSearchPath(src_fldr)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set some parameters
    p.setGravity(0,0,-9.81)
    p.setPhysicsEngineParameter(fixedTimeStep = T_STEP, 
                                     numSolverIterations = 50, 
                                     numSubSteps = 4)
       
    # Load the floor
    p.loadURDF("plane.urdf")
    
    # Load the robot
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    botID = p.loadURDF(urdf, startPos, startOrientation, useFixedBase=1)
    p.changeVisualShape(botID, 0, rgbaColor=[0.0, 0.0, 1, 0.5]) # Just changes the colour of the base
    p.resetBasePositionAndOrientation(botID, startPos, startOrientation) # Not sure this is necessary
    
    # Provide info about the robot joints
    print("Listing joints")
    for i in range(p.getNumJoints(botID)):
        print(p.getJointInfo(botID, i)[0], p.getJointInfo(botID, i)[1].decode('UTF-8'))
    
    # Move the camera to somewhere useful
    botPos, botOrn = p.getBasePositionAndOrientation(botID)
    p.resetDebugVisualizerCamera( cameraDistance=2.5, 
                                    cameraYaw=30.0, 
                                    cameraPitch=-50.0, 
                                    cameraTargetPosition=botPos)

    # Set some initial joint values (in radians) (see joint info printed for the joint numbers)
    p.setJointMotorControlArray(botID, [2, 3], p.POSITION_CONTROL, [0, 2])
    
    # Start time stepping the client
    Step(physicsClientID=physicsClientID, botID=botID)
    
    # End
    Disconnect()

Start()
