import numpy as np
import pybullet as p
import vs068_pb.config as config
import pybullet_data
# import rhinoscriptsyntax as rs
# from compas.rpc import Proxy

## pythonw prevents a window from opening
# with Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\pythonw.exe') as proxy:

#     proxy.run_test()

def run_test():
    import sys
    print("Test", file=sys.stdout)

def load_reach_data():
    import vs068_pb.reachability as reach
    test = reach.Reachability()
    test.load_data()
    test.view_data()
    return test.data

def load_demo():
    from vs068_pb.utils import Disconnect, SetupParams, Step
    try:

        config.DEBUG = False
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

        # Start stepping the sim
        Step(steps=1000000000, cid=physicsClient, botId=botId)
    finally:
        Disconnect()
        return config.prev_pose
