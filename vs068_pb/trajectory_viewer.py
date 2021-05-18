import time
import pybullet as p

from vs068_pb.utils import set_joint_states, loadFloor, Disconnect, quick_load_bot, Camera
import vs068_pb.config as config

import math
import numpy as np


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

def view_trajectory(trajectories, goals=None, success_list=None, visual_fn = lambda q: False, interpolate_gaps=True, loop=True):
    # Disconnect the collision fn
    Disconnect()


    if goals:
        # Create the coloured goal bot
        goalBot, cid = quick_load_bot(p.GUI, collisions=False)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        collision_view = visual_fn(cid, collision_boxes=True)        

        p.changeVisualShape(goalBot, 0, rgbaColor=[0.0, 1.0, 0.0, 0.0])
        for j in range(1, 12):
            p.changeVisualShape(goalBot, j, rgbaColor=[0.0, 1.0, 0.0, 0.3])
        set_joint_states(cid, goalBot, config.info.free_joints, goals[1], [0]*6)

    botId, cid = quick_load_bot(p.GUI, physicsClient=cid)#, physicsClient=cid)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=cid)

    loadFloor(cid)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    while True:
        for count, trajectory in enumerate(trajectories):
            if goals:
                set_joint_states(cid, goalBot, config.info.free_joints, goals[count+1], [0]*6)

                if success_list:
                        if success_list[count]:
                            for j in range(1, 12):
                                p.changeVisualShape(goalBot, j, rgbaColor=[0.0, 1.0, 0.0, 0.3])
                        else:
                            for j in range(1, 12):
                                p.changeVisualShape(goalBot, j, rgbaColor=[1.0, 0.0, 0.0, 0.3])

            times = [trajectory[i]['t'] for i in range(len(trajectory))]
            #print(times)

            start_time = time.time()
            current_t = 0.0
            last_t_id = 0

            while current_t < times[-1]:
                current_t = time.time() - start_time
                
                current_t_id = find_nearest(times, current_t)
                if interpolate_gaps:
                    if current_t_id == last_t_id and current_t_id != len(times)-1:
                        # Some interpolation
                        time_diff_to_current = max(current_t - times[current_t_id], 0.0001)
                        time_diff_to_next = max(0.0001, times[current_t_id+1] - times[current_t_id])
                        time_prop = time_diff_to_current/time_diff_to_next 
                        point_current = [(trajectory[current_t_id+1]['p'][i] - trajectory[current_t_id]['p'][i])*time_prop + trajectory[current_t_id]['p'][i] for i in range(len(trajectory[current_t_id]['p']))]
                    else:
                        point_current = trajectory[current_t_id]['p']
                else:
                    point_current = trajectory[current_t_id]['p']
                
                if current_t_id != last_t_id:
                    set_joint_states(cid, botId, config.info.free_joints, point_current, [0]*6)
                time.sleep(0.1)
                last_t_id = current_t_id

Disconnect()