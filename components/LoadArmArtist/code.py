"""
On boolean true input, load the robot model to be displayed in Rhino
    Args:
        scale_factor: Amount to scale the robot model by
        load: BOOL
"""
        
import os
from scriptcontext import sticky as st

import compas

from compas_ghpython.artists import RobotModelArtist
from compas.robots import LocalPackageMeshLoader
from compas.robots import RobotModel
from compas_fab.robots import RobotSemantics
import compas_fab
from compas_fab.robots import Robot as RobotClass

ghenv.Component.Message = 'LoadArmArtist v0.1'
compas.PRECISION = '12f'

# store robot in component
guid = str(ghenv.Component.InstanceGuid)
robot_key = "robot_" + guid
visual_key = "visual_" + guid

if robot_key not in st:
    st[robot_key] = None

if visual_key not in st:
    st[visual_key] = None

if load:
    os.chdir(r"C:\Users\Sam\OneDrive - University of Leeds\_PhD\ROS\vs068_pb\resources")
    urdf_filename = r'C:\Users\Sam\OneDrive - University of Leeds\_PhD\ROS\vs068_pb\resources\vs068_with_gripper_pybullet.urdf'
    #urdf_filename = compas_fab.get('vs068/vs068_description/urdf/vs068.urdf')
    #srdf_filename = compas_fab.get('vs068/vs068_moveit_config/config/vs068.srdf')
    
    model = RobotModel.from_urdf_file(urdf_filename)
    #semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    
    loader = LocalPackageMeshLoader(compas_fab.get('vs068'), 'vs068_description')
    model.load_geometry(loader)
    
    robot = RobotClass(model)
    
    robot.artist = RobotModelArtist(robot.model)
    st[robot_key] = robot
    robot.scale(scale_factor)
    st[visual_key] = robot.draw()


robot = st[robot_key]
visual = robot.draw_visual()