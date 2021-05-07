import pybullet as p
from vs068_pb.ik_fk import get_valid_ik, getFK_FN
from vs068_pb.utils import quick_load_bot, save_state, restore_state, get_delta_pose_generator, argmin, get_distance, set_joint_states, \
    interval_generator, sample_line, get_difference, Disconnect, loadFloor, randomize, get_pose_distance, uniform_generator, less_than_tol
from vs068_pb.rrt import TreeNode, configs, get_dist_fn
import vs068_pb.config as config
from math import radians, degrees
import time
from random import random
import numpy as np

def get_extend_fn(obstacles=[]):
    # collision_fn, _ = get_collision_fn(obstacles)
    roadmap = []

    def extend_fn(q1, q2, step=0.1):
        path = [q1]
        for q in sample_line(segment=(q1, q2), step_size=step):
            yield q
            # if collision_fn(q):
            #     path = None
            if path is not None:
                #path[-1].children += 1
                roadmap.append((path[-1], q))
                path.append(q)

    return extend_fn, roadmap

##########

# PSEUDOCODE - From Kuffner 2000, "RRT-Connect: An Efficient Approach to Single-Query Path Planning"

# Ta.init(qinit); Tb.init(qgoal);
# for k = 1 to K do
# qrand ← RANDOM CONFIG();
# if not (EXTEND(Ta, qrand) =Trapped) then
# if (CONNECT(Tb, qnew) =Reached) then
# Return PATH(Ta, Tb);
# SWAP(Ta, Tb);

def rrt_connect(current_conf, desired_conf, collision_fn = lambda q: False, tool_space=True, tolerance=0.01, time_limit = 5.0, step = 0.1, n_it = 100, \
        visualise=0, **kwargs):
    config.DEBUG = False
    extend_fn, roadmap = get_extend_fn()

    collisions = 0
    not_collisions = 0

    # Check start/end states
    if collision_fn(current_conf) or collision_fn(desired_conf):
        if collision_fn(current_conf):
            print("Start : {}".format(current_conf))
        if collision_fn(desired_conf):
            print("End : {}".format(desired_conf))
        return [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)], False

    start_time = time.time()
    fk = getFK_FN()
    
    nodes_list = [[TreeNode(current_conf)], [TreeNode(desired_conf)]]

    if tool_space and isinstance(tolerance, (int, float)):
        tolerance = [tolerance, tolerance*10]

    dist_fun = get_dist_fn(tool_space)

    generator = interval_generator(config.lower_lims, config.upper_lims, use_halton=True)

    connect = True

    for counter in range(int(n_it)):
        # Don't go on for too long
        elapsed = time.time() - start_time
        if elapsed > time_limit:
            print("RRTConnect: Timed out at {} seconds! Returning closest".format(elapsed))
            break
        
        if len(nodes_list[0]) < len(nodes_list[1]):
            nodes = nodes_list[0]
            nodes_to = nodes_list[1]
            backwards = False
        else:
            nodes = nodes_list[1]
            nodes_to = nodes_list[0]
            backwards = True
    

        new_conf = next(generator)

        connect = True
        # Get closest in active tree and extend from it
        last, smallest = argmin(lambda n: dist_fun(n, new_conf), nodes)
        for q in extend_fn(last.config, new_conf, step=step):
            if collision_fn(q):
                connect = False
                collisions += 1
                break
            not_collisions += 1
            last = TreeNode(q, parent=last)
            nodes.append(last)

        if connect:
            # Get closest in goal tree and extend to the closest in active
            last_to, smallest = argmin(lambda n: dist_fun(n, last), nodes_to)
            for q in extend_fn(last_to.config, new_conf, step=step):
                if collision_fn(q):
                    connect = False
                    break
                last_to = TreeNode(q, parent=last_to)
                nodes_to.append(last_to)

        if connect:
            break
            
    if connect == True:
        if backwards:
            nodes = nodes_list[0]
            nodes_to = nodes_list[1]
            _ = last
            last = last_to
            last_to = _
        
        for q in configs(last_to.retrace()[::-1]):
            last = TreeNode(q, parent=last)
            nodes.append(last)

        closest = last
        print("RRTConnect: Connecting graph found in {} sec!".format(time.time() - start_time))
    else:
        closest, closest_dist = argmin(lambda n: dist_fun(n, desired_conf), nodes_list[0])

    if config.TEST_COLLISIONS:
        print("{} collisions, {} not collisions".format(collisions, not_collisions))

    ### Plotting
    if visualise != 0:       
        from mpl_toolkits import mplot3d
        import matplotlib.pyplot as plt

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        nodes_rnd = randomize(nodes)

        if visualise == -1:
            visualise = 1
            nodes_rnd = [closest]
        fk = getFK_FN()
        
        for i in range(min(len(nodes), visualise)):
            path = configs(nodes_rnd[i].retrace())
            poses = [fk(q) for q in path]
            x = ([p[0][0] for p in poses])
            y = ([p[0][1] for p in poses])
            z = ([p[0][2] for p in poses])
            c = list(range(len(x)))

            ax.plot3D(x, y, z)

            if visualise == 1:
                ax.scatter3D(x, y, z, c=c, cmap='Reds');

        plt.show()
    #print(closest.retrace())
    return configs(closest.retrace()), connect

if __name__=='__main__':
    goal =  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    rrt_connect((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=-1, tolerance = [0.1, 0.1])
    rrt_connect((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=-1, tool_space=False)
    