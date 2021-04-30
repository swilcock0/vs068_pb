import pybullet as p
import pybullet_data

from vs068_pb.utils import Disconnect, Step, SetupParams
import vs068_pb.config as config
#import vs068_pb.ik_fk as ik_fk


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
Start()
