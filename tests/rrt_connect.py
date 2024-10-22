from vs068_pb.motion.rrt_connect import rrt_connect
from vs068_pb.utils import Disconnect, quick_load_bot, set_joint_states, Camera, loadFloor, interval_generator, create_box_collisions, Pose
from vs068_pb.viewers import view_path
from vs068_pb.viewers import view_trajectory
from vs068_pb.motion.path_to_traj import path_to_traj
from vs068_pb.planning_scene import Scene, Geometry
from vs068_pb.motion.naive_smooth import shortcut, shortcut_relax
import pybullet as p
import vs068_pb.config as config
import time
import math
import sys

## Set up planning scene
MyScene = Scene()

# Boxes
dims = [[0.2, 0.2, 0.2], [0.1, 0.1, 1.2], [0.1, 0.1, 1.2]]
pos = [[0.3,0.3,0.7], [-0.3, -0.3, 0.6], [-0.35, 0.35, 0.6]]

for i in range(len(dims)):
    box = Geometry(physicsClientId=MyScene.physicsClientId)

    box.define_box([pos[i], [0,0,0,1]], dims[i])

    MyScene.add_object(box)

collision_fn = MyScene.initialise_collision_fn()
visual_fn = MyScene.visual_fn


## Define some goals to test to
goals =  [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

generator = interval_generator(config.lower_lims, config.upper_lims, use_halton=True)
print("Finding valid configurations")
for i in range(4):
    next_goal = next(generator)

    while (collision_fn(next_goal) == True):
        next_goal = next(generator)
    goals.append(tuple(next_goal))

goals.append(goals[0])

# Sanity check
for goal in goals:
    assert collision_fn(goal) == False

# Set initial path vars
paths = []
success_list = []
path_section = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
tolerance = 0.01# [0.01, 0.1] 

start_time = time.time()

for i in range(1, len(goals)):
    #print(goals[i-1], goals[i])
    time_done = time.time()
    path_section, success = rrt_connect(path_section[-1], goals[i], n_it = 1000, time_limit = 1.5, tool_space=False, step=0.1, tolerance=tolerance, visualise=0, collision_fn=collision_fn)
    path_section = shortcut(path_section, collision_fn=collision_fn, time_limit=1.5-(time.time()-time_done))
    paths.append(path_section)
    success_list.append(success)
    print(i)

print("")
#print(success_list)

print("Path : {} long, Goals : {} long".format(len(paths), len(goals)-1))
print("Took {:.2f} s, {:%} success rate".format(time.time() - start_time, sum(success_list)/len(success_list)))


view_path(paths, goals, success_list, visual_fn)

print("Converting paths to trajectories")
trajectories = [path_to_traj(paths[i], t_dur=3) for i in range(len(paths))]
view_trajectory(trajectories, goals, success_list, visual_fn)

