import pybullet as p
from vs068_pb.ik_fk import get_valid_ik, getFK_FN
from vs068_pb.utils import quick_load_bot, save_state, restore_state, get_delta_pose_generator, argmin, get_distance, set_joint_states, \
    interval_generator, sample_line, get_difference, Disconnect, loadFloor, randomize, get_pose_distance, uniform_generator, less_than_tol, \
        size_all
import vs068_pb.config as config
from math import radians, degrees
import time
from random import random
import numpy as np

class TreeNode(object):

    def __init__(self, config, parent=None):
        self.fk = getFK_FN()
        self.config = config
        self.parent = parent
        self.pose = []
        #self.pose = []
        self.children = 0

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def getPose(self):
        if self.pose == []:
            self.pose = self.fk(self.config)
        return self.pose

    def __str__(self):
        return self.__class__.__name__ + '(' + str(self.config) + ')'
    __repr__ = __str__


def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))


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


###########

# PSEUDOCODE - From https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378

# Qgoal //region that identifies success
# Counter = 0 //keeps track of iterations
# lim = n //number of iterations algorithm should run for
# G(V,E) //Graph containing edges and vertices, initialized as empty
# While counter < lim:
#     Xnew  = RandomPosition()
#     if IsInObstacle(Xnew) == True: ## Add validation later
#         continue
#     Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
#     Link = Chain(Xnew,Xnearest)
#     G.append(Link)
#     if Xnew in Qgoal:
#         Return G
# Return G

def rrt(current_conf, desired_conf, collision_fn = lambda q: False, tool_space=True, tolerance=0.01, limits=[config.lower_lims, config.upper_lims], time_limit = 1.0, step = 0.01, n_it = 100, \
        visualise=0, greedy_prob = 0.2, return_tree=False, **kwargs):
    config.DEBUG = False
    extend_fn, roadmap = get_extend_fn()

    # Check start/end states
    if collision_fn(current_conf) or collision_fn(desired_conf):
        if collision_fn(current_conf):
            print("Start : {}".format(current_conf))
        if collision_fn(desired_conf):
            print("End : {}".format(desired_conf))
        return [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)], False

    start_time = time.time()
    fk = getFK_FN()
    desired_fk = fk(desired_conf)
    nodes = [TreeNode(current_conf)]
    np.random.seed(int(time.time()))
    closest_dist = 999
    found = False    
    bias = True 

    if tool_space and isinstance(tolerance, (int, float)):
        tolerance = [tolerance, tolerance*10]

    dist_fun = get_dist_fn(tool_space)

    generator = interval_generator(limits[0], limits[1], use_halton=True)

    # Greedy generator
    goal_region_fraction = 0.02 # Proportion of joint range each way
    goal_lim_range = [limits[1][i] - limits[0][i] for i in range(len(limits[0]))]
    goal_lower_lims = [desired_conf[i] - goal_lim_range[i]*goal_region_fraction for i in range(len(limits[0]))]
    goal_upper_lims = [desired_conf[i] + goal_lim_range[i]*goal_region_fraction for i in range(len(limits[0]))]
    generator_goal = interval_generator(goal_lower_lims, goal_upper_lims, use_halton=True)

    rand_num = uniform_generator(1)
    collisions = 0
    not_collisions = 0
    for counter in range(int(n_it)):
        # Don't go on for too long
        elapsed = time.time() - start_time
        if elapsed > time_limit:
            if config.PLANNING_VERBOSE:
                print("RRT: Timed out at {} seconds!".format(elapsed))
            break

        if bias:
            if next(rand_num)[0] < greedy_prob or counter == 0:
                new_conf = desired_conf#next(generator_goal)
            else:
                new_conf = next(generator)
        else:
            new_conf = next(generator)
        
        nodes_select = randomize(nodes)[:min(100, len(nodes))]
        
        last, smallest = argmin(lambda n: dist_fun(n, new_conf), nodes_select)

        for q in extend_fn(last.config, new_conf, step=step):
            if collision_fn(q):
                collisions += 1
                break
            not_collisions += 1
            last = TreeNode(q, parent=last)
            nodes.append(last)

            last_to_desired = dist_fun(last, desired_conf)
            if closest_dist == 999:
                closest_dist = last_to_desired

            if less_than_tol(closest_dist, last_to_desired):
                closest_dist = last_to_desired
                # if closest_dist < 1: # Some kind of annealing but leads to irregular trajectories
                #     step = closest_dist

                if config.DEBUG:
                    print("Closest : {}, step : {}, counter : {}".format(closest_dist, step, counter))

                if less_than_tol(tolerance, dist_fun(last, desired_conf)):
                    if config.PLANNING_VERBOSE:
                        print("RRT: Found a path to goal within tolerance in {} sec!".format(elapsed))
                    #dist_fun(last, desired_conf)
                    found = True
                    break
                    #return configs(last.retrace())  
        if found:
            break
    
    if counter >= n_it-1:
        if config.PLANNING_VERBOSE:
            print("RRT: Maxed out iterations.")

    if config.TEST_COLLISIONS:
        print("{} collisions, {} not collisions".format(collisions, not_collisions))

    if found == True:
        closest = last
    else:
        closest, closest_dist = argmin(lambda n: dist_fun(n, desired_conf), nodes)

    if config.DEBUG:
        print("Closest : {}, step : {}, counter : {}".format(closest_dist, step, counter))

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


    if return_tree:
        return configs(closest.retrace()), found, nodes
    else:
        del(nodes)
        return configs(closest.retrace()), found

def get_dist_fn(tool_space=False):
    from vs068_pb.ik_fk import getFK_FN

    fk = getFK_FN()

    if tool_space:
        def fn(node_a, node_b=-1):
            if isinstance(node_b, TreeNode):
                #print("Node")
                test_pose = node_b.getPose()
            elif isinstance(node_b, int):
                #print("int")
                test_pose = desired_fk
            elif isinstance(node_b, list):
                #print("list")
                test_pose = fk(node_b)
            elif isinstance(node_b, np.ndarray):
                #print("NP Array")
                test_pose = fk(node_b)
            else:
                #print(type(node_b))
                test_pose = fk(node_b)

            distance = get_pose_distance(node_a.getPose(), test_pose)
            return distance #2*distance[0] + distance[1]
    else:
        def fn(conf_a, conf_b):
            if isinstance(conf_a, TreeNode):
                conf_a = conf_a.config
            if isinstance(conf_b, TreeNode):
                conf_b = conf_b.config 

            #print(conf_a, conf_b)
            return get_distance(conf_a, conf_b)
            #return max([abs(conf_a[q] - conf_b[q]) for q in range(len(conf_a))])
    return fn

if __name__=='__main__':
    dims = [[0.2, 0.2, 0.2], [0.1, 0.1, 1.2], [0.1, 0.1, 1.2]]
    pos = [[0.3,0.3,0.7], [-0.25, -0.25, 0.6], [-0.25, 0.25, 0.6]]
    collision_fn, _ = create_box_collisions(dims, pos)

    goal =  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    rrt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=100, tolerance = [0.1, 0.1])


    generator = interval_generator(limits[0], limits[1], use_halton=True)

    goal = next(generator)
    while (collision_fn(goal) == True):
        goal = next(generator)

    rrt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=100, tool_space=False, collision_fn=collision_fn)
    

