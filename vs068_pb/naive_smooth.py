import time
import numpy as np
from vs068_pb.utils import sample_line, uniform_generator, get_distance, spline_uni, smooth_spline
import vs068_pb.config as config
'''
See https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6165411/ for smoothing rather than shortcutting based on NURBS 
'''
'''
See Adaptive Partial Shortcuts: Path Optimizationfor Industrial Robotics
https://click.endnote.com/viewer?doi=10.1007%2Fs10846-016-0437-x&token=WzI5NjU3MDEsIjEwLjEwMDcvczEwODQ2LTAxNi0wNDM3LXgiXQ.LHNOZA9P0qP_L8IMu7I21rf9eYQ
for categorisation of shortcutting techniques
'''

def shortcut(path, collision_fn = lambda q: False, time_limit=0.5, step=0.1):
    ''' Naive smoother based on random shortcutting '''
    if len(path) < 3:
        return path
    start = path[0]
    end = path[-1]
    np.random.seed(int(time.time()))
    rand_num = uniform_generator(1)
    path_current = path
    steps = [get_distance(path[i], path[i+1]) for i in range(0, len(path)-1)]
    average = np.mean(steps)
    total = np.sum(steps)
    step_orig = np.mean(steps)
    #if np.isclose(step_orig, 0.1, 
    #print(steps)
    #print(step_orig)
    shortcuts = 0
    start_time = time.time()
    
    while time.time() - start_time <= time_limit:
        collide = False
        node_a = int(next(rand_num)*len(path_current))
        node_b = int(next(rand_num)*len(path_current))
        node_a, node_b = min(node_a, node_b), max(node_a, node_b)

        # Change node b to find nearest?
        if node_b-node_a <= 5:
            continue

        if get_distance(path_current[node_a], path_current[node_b]) > average*(node_b - node_a):
            continue

        joining_path = list(sample_line(segment=(path_current[node_a], path_current[node_b]), step_size=step)) 

        for node in joining_path:
            if collision_fn(node):
                collide = True
                break

        if collide:
            continue
        else:
            # TODO : Check node_a/node_b not ends
            path_current = path_current[:node_a] + joining_path + path_current[node_b+1:]
            shortcuts += 1

    steps = [get_distance(path_current[i], path_current[i+1]) for i in range(1, len(path_current)-1)]
    total_new = np.sum(steps)
    if config.PLANNING_VERBOSE:
        print("Path length {}. reduced by {:.2%} with {} shortcuts made".format(total_new, (total-total_new)/total, shortcuts))
    return path_current


def shortcut_relax(path, collision_fn = lambda q: False, time_limit=np.Inf, step=0.1, **kwargs):
    ''' End to end shortcutting - see paper on RRT-Relax '''
    ''' Takes a long time! Worst case O(n^2) '''
    if len(path) < 3:
        return path
    start = path[0]
    end = path[-1]
    path_current = path
    steps = [get_distance(path[i], path[i+1]) for i in range(0, len(path)-1)]
    average = np.mean(steps)
    total = np.sum(steps)
    step_orig = np.mean(steps)

    shortcuts = 0
    start_time = time.time()
    
    p_l = len(path_current)
    node_a = 0

    while(node_a < p_l-2 and time.time()-start_time < time_limit):
        node_a += 1
        node_b = 1
        while(node_a < p_l-2 and node_b < p_l and time.time()-start_time < time_limit):
            node_b += 1
            collide = False
            p_l = len(path_current)
            node_a_tmp = node_a
            node_b_tmp = p_l - node_b

            node_a_tmp, node_b_tmp = min(node_a_tmp, node_b_tmp), max(node_a_tmp, node_b_tmp)
            if abs(node_b_tmp-node_a_tmp) <= 2:
                continue
            
            #print(p_l, node_a, node_a_tmp, node_b, node_b_tmp)
            if get_distance(path_current[node_a_tmp], path_current[node_b_tmp]) > average*(node_b_tmp - node_a_tmp):
                continue

            joining_path = list(sample_line(segment=(path_current[node_a_tmp], path_current[node_b_tmp]), step_size=step)) 

            for node in joining_path:
                if collision_fn(node):
                    collide = True
                    break

            if collide:
                continue
            else:
                path_current = path_current[:node_a_tmp] + joining_path + path_current[node_b_tmp+1:]
                shortcuts += 1
                p_l = len(path_current)
            

    steps = [get_distance(path_current[i], path_current[i+1]) for i in range(1, len(path_current)-1)]
    total_new = np.sum(steps)
    if config.PLANNING_VERBOSE:
        print("Path length {}, relaxed by {:.2%} with {} shortcuts made in {} s".format(total_new, (total-total_new)/total, shortcuts, round(time.time()-start_time, 2)))
    return path_current

def smooth(path):
    path_feed = list(path)
    path_new = [[0 for q in path[0]] for p in path]
    for q in range(len(path[0])):
        points = [path[i][q] for i in range(len(path))]
        points = spline_uni(points)

        for i in range(len(points)):
            path_new[i][q] = points[i]
        
    return path
