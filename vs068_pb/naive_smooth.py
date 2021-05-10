import time
import numpy as np
from vs068_pb.utils import sample_line, uniform_generator, get_distance


def smooth(path, collision_fn = lambda q: False, time_limit=0.5, step=0.1):
    ''' Naive smoother based on random shortcutting '''
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

    start_time = time.time()
    
    while time.time() - start_time <= time_limit:
        collide = False
        node_a = int(next(rand_num)*len(path_current))
        node_b = int(next(rand_num)*len(path_current))
        node_a, node_b = min(node_a, node_b), max(node_a, node_b)

        # Change node b to find nearest?
        if node_b-node_a == 1 or node_b-node_a == 0:
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
            path_current = path_current[:node_a] + joining_path + path_current[node_b+1:]

    steps = [get_distance(path_current[i], path_current[i+1]) for i in range(1, len(path_current)-1)]
    total_new = np.sum(steps)
    print("Path length reduced by {:.2%}".format((total-total_new)/total))
    return path_current
