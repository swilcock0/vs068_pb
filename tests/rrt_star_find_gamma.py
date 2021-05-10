from vs068_pb.rrt_star import rrt_star
from pstats import SortKey
from vs068_pb.utils import Disconnect, quick_load_bot, set_joint_states, Camera, loadFloor, interval_generator, create_box_collisions, Pose,\
    size_all, flip_dict
from vs068_pb.path_viewer import view_path
import pybullet as p
import vs068_pb.config as config
import time
import math
import sys
import numpy as np


dims = [[0.2, 0.2, 0.2], [0.1, 0.1, 1.2], [0.1, 0.1, 1.2]]
pos = [[0.3,0.3,0.7], [-0.25, -0.25, 0.6], [-0.25, 0.25, 0.6]]
collision_fn, visual_fn = create_box_collisions(dims, pos, safety=0.03)

goals =  [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

generator = interval_generator(config.lower_lims, config.upper_lims, use_halton=True)
for i in range(100):
    next_goal = next(generator)

    while (collision_fn(next_goal) == True):
        next_goal = next(generator)
    goals.append(tuple(next_goal))

goals.append(goals[0])

# Sanity check
for goal in goals:
    assert collision_fn(goal) == False

# Set initial path vars

success_gamma = {}

for gamma in np.linspace(0.1, 5, 20):
    paths = []
    success_list = []
    path_section = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
    tolerance = 0.1 # Change me if you want!
    for i in range(1, len(goals)):
        path_section, success = rrt_star(path_section[-1], goals[i], n_it = 99999, time_limit = 1.0, gamma=gamma, tool_space=False, step=0.1, tolerance=tolerance, collision_fn=collision_fn)
        if (len(path_section) < 3):
            print(path_section)
            config.TEST_COLLISIONS_VERBOSE = True
        else:
            config.TEST_COLLISIONS_VERBOSE = False
        paths.append(path_section)
        success_list.append(success)

    print("Gamma {}, {:%} success rate".format(gamma, sum(success_list)/len(success_list)))
    success_gamma[gamma] = sum(success_list)/len(success_list)

print(success_gamma)
print(max(success_gamma, key=success_gamma.get), ':', success_gamma[max(success_gamma, key=success_gamma.get)])
