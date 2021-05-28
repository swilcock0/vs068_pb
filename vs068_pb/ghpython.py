import sys
import os
import vs068_pb.config as config
sys.stdout = open(os.path.join(config.src_fldr, "ghpython_log.txt"), 'w') # Change to 'a' for append

from vs068_pb.utils import quick_load_bot, set_joint_states
import numpy as np
import pybullet as p

from vs068_pb.planning_scene import Scene, Geometry
import pybullet_data
import tempfile
import atexit
import time
from datetime import datetime

load_time = datetime.now()
dt_string = load_time.strftime("%d/%m/%Y %H:%M:%S")
print("Logfile time: ", dt_string)


# Example usage in Grasshopper:
# import rhinoscriptsyntax as rs
# from compas.rpc import Proxy

# with Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\python.exe') as proxy:
#     proxy.run_test()

# ## Or, without context managers
# proxy = Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\python.exe')
# proxy.run_test()
# proxy.stop_server()


@atexit.register
def clean_stdout():
    # Ensure we clean up the logfile when the module is unloaded
    print("Closing logfile...")
    sys.stdout.close()    

@atexit.register
def clean_temp():
    if config.tmp:
        config.tmp.cleanup()
    #config.tmp.close()

def run_test():
    import sys
    print("Test", file=sys.stdout)

def load_reach_data():
    print("Loading reachability data...")
    start_time = time.time()
    import vs068_pb.reachability as reach
    test = reach.Reachability()
    test.load_data()
    test.view_data()
    print("Reach data loaded in {} seconds".format(int(time.time() - start_time)))
    return test.data

def load_demo():
    from vs068_pb.utils import Disconnect, SetupParams, Step
    try:
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

def load_mesh(file_id, pos=[1,0,0], concavity=False):  
    print("Loading mesh from ", file_id)
    scene = config.SCENE_STORAGE
    
    if scene:
        mesh_geo = Geometry(physicsClientId=scene.physicsClientId, mass=100.0, safety_margin=1.05)
        mesh_geo.define_mesh(file_id, pose_centre=[pos, [0,0,0,1]], scale=1, concavity=concavity)
        scene.add_object(mesh_geo)
        #scene.initialise_allowed()

def clear_scene():
    print("Clearing scene")
    if config.SCENE_STORAGE:
        config.SCENE_STORAGE.clear_all()
        display_contacts()

def get_scene_objects():
    list_out = []
    if config.SCENE_STORAGE:
        for cobject in config.SCENE_STORAGE.collision_objects:
            list_out.append(str(cobject) + " : " + config.SCENE_STORAGE.collision_objects[cobject].__repr__())
    return list_out

def get_full_scene():
    if config.SCENE_STORAGE:
        scene_dict = config.SCENE_STORAGE.get_scene_dict()
    else:
        scene_dict = {}

    scene_out = []
    for item in scene_dict:
        scene_out.append([item, scene_dict[item]])
    return scene_out

def Disconnect():
    for i in range(10):
        try:
            p.disconnect(i)
        except:
            pass

def set_realtime():
    print("Setting to realtime mode")
    p.setGravity(0.0, 0.0, -9.81, config.SCENE_STORAGE.physicsClientId)
    p.setRealTimeSimulation(1, config.SCENE_STORAGE.physicsClientId)

def step(n_it=1000):
    print("Stepping {} times".format(n_it))
    p.setGravity(0.0, 0.0, -9.81, config.SCENE_STORAGE.physicsClientId)
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
    print("Joint positions set to ", str(q_vals))
    if config.SCENE_STORAGE:
        scene = config.SCENE_STORAGE

        set_joint_states(scene.physicsClientId, scene.botId, config.info.free_joints + config.FINGER_JOINTS, q_vals, [0]*len(q_vals))

def check_contacts():
    if config.SCENE_STORAGE:
        print("Contacts: ")
        scene = config.SCENE_STORAGE
        p.setGravity(0.0, 0.0, 0.0, scene.physicsClientId)
        p.stepSimulation()
        contacts = p.getContactPoints(physicsClientId=scene.physicsClientId)
        for contact in contacts:
            print(contact)
        p.setGravity(0.0, 0.0, -9.81, scene.physicsClientId)
        print("")
        return len(contacts)
    else:
        return 0

# def plan_to_config():
#     if config.SCENE_STORAGE:
#         scene = config.SCENE_STORAGE

#         scene.

def clear_contacts():
    if config.SCENE_STORAGE:
        scene = config.SCENE_STORAGE

        if config.CONTACT_POINT_STORAGE:
            
            for point in config.CONTACT_POINT_STORAGE:
                p.removeBody(point, physicsClientId=scene.physicsClientId)
                #p.removeUserDebugItem(point, physicsClientId=scene.physicsClientId)


        if config.CONTACT_LINE_STORAGE:
            
            for line in config.CONTACT_LINE_STORAGE:
                #p.removeBody(point, physicsClientId=scene.physicsClientId)
                p.removeUserDebugItem(line, physicsClientId=scene.physicsClientId)


        config.CONTACT_POINT_STORAGE = []
        config.CONTACT_LINE_STORAGE = []

def display_contacts(step_sim=False):
    if config.SCENE_STORAGE:
        scene = config.SCENE_STORAGE
        clear_contacts()
        p.setGravity(0.0, 0.0, 0.0, scene.physicsClientId)
        if step_sim:
            p.stepSimulation()
        contacts = p.getContactPoints(physicsClientId=scene.physicsClientId)
        for contact in contacts:
            if not([contact[3], contact[4]] in config.NEVER_COLLIDE_NUMS and contact[1] == contact[2]):
                sphere_coll_vis = p.createVisualShape(p.GEOM_SPHERE, 
                                physicsClientId=scene.physicsClientId, 
                                radius = 0.03, 
                                rgbaColor=[1,0,0,0.5]
                                )

                id_collision = p.createMultiBody(baseMass=0, baseVisualShapeIndex=sphere_coll_vis, basePosition = contact[5], physicsClientId=scene.physicsClientId)
                config.CONTACT_POINT_STORAGE.append(id_collision)

                norm_vector = contact[7]
                norm_vector = [n*0.5 for n in norm_vector]
                lineTo = [contact[6][i] + norm_vector[i] for i in range(3)]
                lineId = p.addUserDebugLine(contact[6], lineTo, lineColorRGB=[1,0,0,1], lineWidth=0.1, lifeTime=0, physicsClientId=scene.physicsClientId)
                config.CONTACT_LINE_STORAGE.append(lineId)
                lineTo = [contact[6][i] - norm_vector[i] for i in range(3)]         
                lineId = p.addUserDebugLine(contact[6], lineTo, lineColorRGB=[1,0,0,1], lineWidth=0.1, lifeTime=0, physicsClientId=scene.physicsClientId)       
                config.CONTACT_LINE_STORAGE.append(lineId)
        return len(contacts)
    else:
        return 0

def get_disassembly():
    from vs068_pb.disassembly import Assembly
    test = Assembly()
    elements, directions = test.disassemble_loosest()
    return elements,directions


# if __name__ == '__main__':
#     load_scene()