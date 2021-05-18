from vs068_pb.motion.naive_smooth import shortcut
from vs068_pb.motion.rrt_star import rrt_star
from vs068_pb.utils import Disconnect, quick_load_bot, set_joint_states, Camera, loadFloor, interval_generator, create_box_collisions, Pose,\
    size_all
from vs068_pb.planning_scene import Scene, Geometry
from vs068_pb.viewers import view_path
import pybullet as p
import vs068_pb.config as config
import time
import math
import sys
import numpy as np


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

goals =  [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

generator = interval_generator(config.lower_lims, config.upper_lims, use_halton=True)
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
tolerance = 0.1 # Change me if you want!

start_time = time.time()

for i in range(1, len(goals)):
    s_time = time.time()
    path_section, success = rrt_star(path_section[-1], goals[i], n_it = 99999, time_limit = 0.5, tool_space=False, step=0.1, tolerance=tolerance, collision_fn=collision_fn)
    if (len(path_section) < 3):
        print(path_section)
        config.TEST_COLLISIONS_VERBOSE = True
    else:
        config.TEST_COLLISIONS_VERBOSE = False
        
    path_section = shortcut(path_section, collision_fn=collision_fn, time_limit=1.5-(time.time()-s_time))
    paths.append(path_section)
    success_list.append(success)
    print(i)

print("")
#print(success_list)

print("Path : {} long, Goals : {} long".format(len(paths), len(goals)-1))
print("Took {:.2f} s, {:%} success rate".format(time.time() - start_time, sum(success_list)/len(success_list)))

view_path(paths, goals, success_list, visual_fn)
