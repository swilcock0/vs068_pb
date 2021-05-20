from vs068_pb.utils import quick_load_bot, set_joint_states
import numpy as np
import pybullet as p
import vs068_pb.config as config
from vs068_pb.planning_scene import Scene, Geometry
import pybullet_data
import tempfile
import os

#tmp = tempfile.TemporaryDirectory()
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

def is_connected():
    return p.isConnected() == 1

def load_scene(load_floor=False, quiet=False):
    Disconnect()
    botId, cid = quick_load_bot(p.GUI, quiet=quiet)

    scene = Scene(cid, botId, quiet=quiet)

    if load_floor:
        scene.add_floor()

    config.SCENE_STORAGE = scene

    return botId, cid

def get_temp_obj_path():
    config.tmp = tempfile.TemporaryDirectory()
    return os.path.join(config.tmp.name, "temp.obj")

def clean_temp():
    config.tmp.cleanup()
    #config.tmp.close()

def load_mesh(file_id, pos=[1,0,0], concavity=False):
    import time
    import os    

    scene = config.SCENE_STORAGE
    
    if scene:
        mesh_geo = Geometry(physicsClientId=scene.physicsClientId, mass=100.0)
        mesh_geo.define_mesh(file_id, pose_centre=[pos, [0,0,0,1]], scale=1, concavity=concavity)
        scene.add_object(mesh_geo)
        #scene.initialise_allowed()

def clear_scene():
    if config.SCENE_STORAGE:
        config.SCENE_STORAGE.clear_all()

def get_scene_objects():
    list_out = []
    if config.SCENE_STORAGE:
        for cobject in config.SCENE_STORAGE.collision_objects:
            list_out.append(str(cobject) + " : " + config.SCENE_STORAGE.collision_objects[cobject].__repr__())
    return list_out

def Disconnect():
    for i in range(10):
        try:
            p.disconnect(i)
        except:
            pass

def set_realtime():
    p.setGravity(0.0, 0.0, -9.81, config.SCENE_STORAGE.physicsClientId)
    p.setRealTimeSimulation(1, config.SCENE_STORAGE.physicsClientId)

def step(n_it=1000):
    p.setGravity(0.0, 0.0, -9.81, config.SCENE_STORAGE.physicsClientId)
    import time
    for n in range(n_it):
        if is_connected():
            config.SCENE_STORAGE.step()
            config.SCENE_STORAGE.match_all_poses()
            time.sleep(1./240.)
    return [q[0] for q in p.getJointStates(config.SCENE_STORAGE.botId, config.FINGER_JOINTS, config.SCENE_STORAGE.physicsClientId)]

def get_joint_positions():
    if config.SCENE_STORAGE:
        scene = config.SCENE_STORAGE
        return [q[0] for q in p.getJointStates(config.SCENE_STORAGE.botId, config.info.free_joints + config.FINGER_JOINTS, config.SCENE_STORAGE.physicsClientId)]
    else:
        return []

def set_joint_positions(q_vals):
    if config.SCENE_STORAGE:
        scene = config.SCENE_STORAGE

        set_joint_states(scene.physicsClientId, scene.botId, config.info.free_joints + config.FINGER_JOINTS, q_vals, [0]*len(q_vals))