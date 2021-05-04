# PSEUDOCODE - From https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
# 
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


import pybullet as p
from vs068_pb.ik_fk import get_valid_ik, getFK_FN
from vs068_pb.utils import quick_load_bot, save_state, restore_state, get_delta_pose_generator, argmin, get_distance, set_joint_states, \
    interval_generator, sample_line, get_difference, Disconnect, loadFloor, randomize, get_pose_distance
import vs068_pb.config as config
from math import radians, degrees
import time
from random import random
import numpy as np

class TreeNode(object):

    def __init__(self, config, parent=None):
        fk = getFK_FN()
        self.config = config
        self.parent = parent
        self.pose = fk(config)
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

    # def draw(self, env, color=apply_alpha(RED, alpha=0.5)):
    #     # https://github.mit.edu/caelan/lis-openrave
    #     from manipulation.primitives.display import draw_node, draw_edge
    #     self.node_handle = draw_node(env, self.config, color=color)
    #     if self.parent is not None:
    #         self.edge_handle = draw_edge(
    #             env, self.config, self.parent.config, color=color)

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'
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

# https://github.com/caelan/motion-planners/blob/master/motion_planners/rrt.py


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

def rrt(current_conf, desired_conf, tool_space=True, tolerance=0.01, time_limit = 5.0, n_it = 100, visualise=0, **kwargs):
    config.DEBUG = False
    extend_fn, roadmap = get_extend_fn()

    start_time = time.time()
    tolerance = 0.1
    fk = getFK_FN()
    desired_fk = fk(desired_conf)
    nodes = [TreeNode(current_conf)]
    np.random.seed(int(time.time()))
    step = 0.5
    closest_dist = 999
    found = False

    #if tool_space and isinstance(tolerance, float):
    #    tolerance = [tolerance]*2

    def get_dist_fn():
        if tool_space:
            def fn(node_a, node_b=-1):
                if isinstance(node_b, TreeNode):
                    #print("Node")
                    test_pose = node_b.pose
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
                    print(type(node_b))
                    test_pose = fk(node_b)

                distance = get_pose_distance(node_a.pose, test_pose)
                return distance[0] #2*distance[0] + distance[1]
        else:
            def fn(conf_a, conf_b):
                if isinstance(conf_a, TreeNode):
                    conf_a = conf_a.config
                if isinstance(conf_b, TreeNode):
                    conf_b = conf_b.config 
                return get_distance(conf_a, conf_b)
                #return max([abs(conf_a[q] - conf_b[q]) for q in range(len(conf_a))])
        return fn

    dist_fun = get_dist_fn()

    generator = interval_generator(config.lower_lims, config.upper_lims, use_halton=False)
    #generator = get_delta_pose_generator(epsilon=-1, angle=radians(180)) # For tool space planning

    for counter in range(int(n_it)):
        # Don't go on for too long
        elapsed = time.time() - start_time
        if elapsed > time_limit:
            print("RRT: Timed out at {} seconds!".format(elapsed))
            break

        new_conf = next(generator)
        
        nodes_select = randomize(nodes)[:min(500, len(nodes))]
        
        last, smallest = argmin(lambda n: dist_fun(n, new_conf), nodes_select)

        for q in extend_fn(last.config, new_conf, step=step):
            # if collision_fn(q):
            #     break
            last = TreeNode(q, parent=last)
            nodes.append(last)

            last_to_desired = dist_fun(last, desired_conf)

            if last_to_desired < closest_dist:
                closest_dist = last_to_desired

                if closest_dist < 1: # Kind of annealing
                    step = closest_dist

                if config.DEBUG:
                    print("Closest : {}, step : {}, counter : {}".format(closest_dist, step, counter))

                if dist_fun(last, desired_conf) < tolerance:
                    print("RRT: Found a path to goal within tolerance in {} sec!".format(elapsed))
                    dist_fun(last, desired_conf)
                    found = True
                    break
                    #return configs(last.retrace())  
        if found:
            break

    if found == True:
        closest = last
    else:
        closest, closest_dist = argmin(lambda n: dist_fun(n, desired_conf), nodes)

    if config.DEBUG:
        print("Closest : {}, step : {}, counter : {}".format(closest_dist, step, counter))

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
        
        for i in range(visualise):
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
        #input("Displaying plot")

    return configs(closest.retrace())

if __name__=='__main__':
    goal =  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    rrt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=100)
    rrt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=100, tool_space=False)