from vs068_pb.rrt import TreeNode, configs, get_dist_fn, get_extend_fn
import vs068_pb.config as config
from vs068_pb.utils import quick_load_bot, save_state, restore_state, get_delta_pose_generator, argmin, get_distance, set_joint_states, \
    interval_generator, sample_line, get_difference, Disconnect, loadFloor, randomize, get_pose_distance, uniform_generator, less_than_tol, \
    size_all
from vs068_pb.ik_fk import getFK_FN

import numpy as np
import time
import numpy as np

''' See original - "Sampling-based algorithms for optimal motion planning" Karaman, Frazolli 2011 '''


class OptimalNode(object):
    def __init__(self, config, parent=None, d=0, path=[], iteration=None):
        self.fk = getFK_FN()
        self.config = config
        self.parent = parent
        self.children = set()
        self.d = d
        self.path = path
        if parent is not None:
            self.cost = parent.cost + d
            self.parent.children.add(self)
        else:
            self.cost = d
        self.solution = False
        self.creation = iteration
        self.last_rewire = iteration
        self.pose = []

    def set_solution(self, solution):
        if self.solution is solution:
            return
        self.solution = solution
        if self.parent is not None:
            self.parent.set_solution(solution)

    def retrace(self):
        if self.parent is None:
            return self.path + [self.config]
        return self.parent.retrace() + self.path + [self.config]

    def rewire(self, parent, d, path, iteration=None):
        if self.solution:
            self.parent.set_solution(False)
        self.parent.children.remove(self)
        self.parent = parent
        self.parent.children.add(self)
        if self.solution:
            self.parent.set_solution(True)
        self.d = d
        self.path = path
        self.update()
        self.last_rewire = iteration
    
    def update(self):
        self.cost = self.parent.cost + self.d
        for n in self.children:
            n.update()

    def getPose(self):
        if self.pose == []:
            self.pose = self.fk(self.config)
        return self.pose


def safe_path(sequence, collision):
    path = []
    for q in sequence:
        if collision(q):
            break
        path.append(q)
    return path

# RRT* Pseudo Code - https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
# Rad = r
# G(V,E) //Graph containing edges and vertices
# For itr in range(0…n)
#     Xnew = RandomPosition()
#     If Obstacle(Xnew) == True, try again
#     Xnearest = Nearest(G(V,E),Xnew)
#     Cost(Xnew) = Distance(Xnew,Xnearest)
#     Xclosest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
#     Link = Chain(Xnew,Xclosest)
#     For x’ in Xneighbors
#         If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
#             Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
#             Parent(x’) = Xnew
#             G += {Xnew,x’}
#     G += Link 
# Return G

# Finding neighbours within radius:
# neighbors = filter(lambda n: dist_fun(n.config, new.config) < radius, nodes)

# Finding k nearest neighbours:
# k = 10
# neighbours = sorted(nodes, key=lambda n: dist_fun(n.config, new.config))
# neighbours = neighbours[:k]


#From tests, gamma values of 0.615 and 1.65 are indicated

def rrt_star(current_conf, desired_conf, collision_fn = lambda q: False, informed=True, tool_space=False, tolerance=0.01, time_limit = 1.0, step = 0.01, n_it = 100, \
        visualise=0, greedy_prob = 0.2, limits=[config.lower_lims, config.upper_lims], return_tree=False, gamma = 1.65, rad=None, **kwargs):
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
    nodes = [OptimalNode(current_conf)]
    np.random.seed(int(time.time()))
    closest_dist = 999
    closest = None
    bias = True
    closest = None
    power = 1/len(current_conf)
    found = False

    if tool_space and isinstance(tolerance, (int, float)):
        tolerance = [tolerance, tolerance*10]
        EPSILON = tolerance[0]
    else:
        EPSILON = tolerance

    dist_fun = get_dist_fn(tool_space)

    generator = interval_generator(limits[0], limits[1], use_halton=True)

    # Greedy generator
    goal_region_fraction = 0.05 # Proportion of joint range each way
    goal_lim_range = [limits[1][i] - limits[0][i] for i in range(len(limits[0]))]
    goal_lower_lims = [desired_conf[i] - goal_lim_range[i]*goal_region_fraction for i in range(len(limits[0]))]
    goal_upper_lims = [desired_conf[i] + goal_lim_range[i]*goal_region_fraction for i in range(len(limits[0]))]
    generator_goal = interval_generator(goal_lower_lims, goal_upper_lims, use_halton=True)

    rand_num = uniform_generator(1)
    collisions = 0
    not_collisions = 0
    for counter in range(int(n_it)):
        if rad == None:
            radius = gamma*(np.log(len(nodes))/len(nodes))**power
        else:
            radius = max(gamma*(np.log(len(nodes))/len(nodes))**power, rad)
        # Don't go on for too long
        elapsed = time.time() - start_time
        if elapsed > time_limit:
            print("RRT*: Timed out at {} seconds!".format(elapsed))
            break

        if bias:
            if next(rand_num)[0] < greedy_prob or counter == 0:
                if counter == 0 or next(rand_num)[0] < 0.5:
                    new_conf = desired_conf
                else:
                    new_conf = next(generator_goal)
            else:
                new_conf = next(generator)
        else:
            new_conf = next(generator)
    
        if informed and (closest is not None) and (dist_fun(current_conf, new_conf) + dist_fun(desired_conf, new_conf) >= closest.cost):
            continue

        nodes_select = nodes# randomize(nodes)[:min(1000, len(nodes))]
        

        nearest, smallest = argmin(lambda n: dist_fun(n, new_conf), nodes_select)
        path = safe_path(extend_fn(nearest.config, new_conf), collision_fn)

        if len(path) == 0:
            continue

        new = OptimalNode(path[-1], parent=nearest, d=dist_fun(
            nearest.config, path[-1]), path=path[:-1], iteration=counter)
        # if safe and do_goal:
        if (dist_fun(new.config, desired_conf) < EPSILON):
            found = True
            closest = new
            closest.set_solution(True)
            closest_dist = dist_fun(closest.config, desired_conf)

        neighbors = filter(lambda n: dist_fun(n.config, new.config) < radius, nodes)
        nodes.append(new)
        #print("{} neighbours".format(len(list(neighbors))))

        if found:
            for i in range(2):
                for n in neighbors:
                    d = dist_fun(n.config, new.config)
                    if (n.cost + d) < new.cost:
                        path = safe_path(extend_fn(n.config, new.config), collision_fn)
                        if (len(path) != 0) and (dist_fun(new.config, path[-1]) < EPSILON):
                            new.rewire(n, d, path[:-1], iteration=counter)

    if counter >= n_it-1:
        print("RRT*: Maxed out iterations.")
    
    if config.TEST_COLLISIONS:
        print("{} collisions, {} not collisions".format(collisions, not_collisions))

    if closest is None:
        closest, closest_dist = argmin(lambda n: dist_fun(n, desired_conf), nodes)

    if less_than_tol(tolerance, dist_fun(closest, desired_conf)):
        if elapsed > time_limit:
            print("RRT*: Found a path to goal within tolerance!")
        else:
            print("RRT*: Found a path to goal within tolerance in {} sec!".format(elapsed))
        found = True
    else:
        print("RRT*: Not found a solution")
        found = False

    print("Distance : {}".format(closest_dist))
    print("Cost : {}".format(closest.cost))


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
        return closest.retrace(), found, nodes
    else:
        del(nodes)
        return closest.retrace(), found

def get_dist_fn(tool_space=False):
    from vs068_pb.ik_fk import getFK_FN

    fk = getFK_FN()

    if tool_space:
        def fn(node_a, node_b=-1):
            if isinstance(node_b, OptimalNode):
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
            if isinstance(conf_a, OptimalNode):
                conf_a = conf_a.config
            if isinstance(conf_b, OptimalNode):
                conf_b = conf_b.config 

            #print(conf_a, conf_b)
            return get_distance(conf_a, conf_b)
            #return max([abs(conf_a[q] - conf_b[q]) for q in range(len(conf_a))])
    return fn