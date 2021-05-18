import time
import pybullet as p

from vs068_pb.utils import set_joint_states, loadFloor, Disconnect, quick_load_bot, Camera, DrawEEFLine, random_brick_gen
import vs068_pb.config as config

import math
import numpy as np

def view_path(paths, goals=None, success_list=None, visual_fn = lambda q: False, loop=True):
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

    botId, cid = quick_load_bot(p.GUI, physicsClient=cid)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=cid)

    loadFloor(cid)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Set up loop vars
    events = p.getKeyboardEvents(cid)
    time_anim = 5.0
    qKey = ord('q')
    xKey = ord('x')
    cKey = ord('c')
    mKey = ord('m')
    pKey = ord('p')
    upKey = p.B3G_UP_ARROW
    downKey = p.B3G_DOWN_ARROW
    spaceKey = p.B3G_SPACE
    start_time = time.time()
    flag = False
    time_anim_bak = 101.0
    space_time = time.time()
    m_time = time.time()
    num_delays = 0
    vis_col_state = True
    path_len_ave = np.mean([len(path) for path in paths])

    ''' TODO : Add non-looping version? '''
    while qKey not in events and xKey not in events:
        for count, path in enumerate(paths):
            if goals:
                set_joint_states(cid, goalBot, config.info.free_joints, goals[count+1], [0]*6)

                if success_list:
                    if success_list[count]:
                        for j in range(1, 12):
                            p.changeVisualShape(goalBot, j, rgbaColor=[0.0, 1.0, 0.0, 0.3])
                    else:
                        for j in range(1, 12):
                            p.changeVisualShape(goalBot, j, rgbaColor=[1.0, 0.0, 0.0, 0.3])

            for q in path:
                if time.time() - start_time > 1.3*(time_anim/path_len_ave):
                    # Speed up lagging depiction
                    #print(time.time() - start_time)
                    #print(num_delays)
                    if num_delays == 0:
                        num_delays = int((time.time()-start_time)/(time_anim/path_len_ave))
                        start_time = time.time()
                    else:
                        num_delays -= 1
                    
                    continue

                start_time = time.time() 
                if time_anim < 100:
                    set_joint_states(cid, botId, config.info.free_joints, q, [0]*6)
                time.sleep(time_anim/path_len_ave)  

                # Check keyboard
                events = p.getKeyboardEvents(cid)
                if xKey in events:
                    events.update({qKey : 1}) # TODO (swilcock0) : Fix this test
                    break
                elif upKey in events:
                    time_anim = max(math.log(time_anim+1), time_anim-1)
                    #print(time_anim)
                elif downKey in events:
                    time_anim += 1
                    #print(time_anim)
                elif pKey in events:
                    print("Plotting graph")
                    graph_path(path)
                elif spaceKey in events and time.time() - space_time > 0.5: #Prevent bounce
                    space_time = time.time()
                    tmp = time_anim_bak
                    time_anim_bak = time_anim
                    time_anim = tmp
                    del events[spaceKey]
                    while spaceKey not in events:
                        time.sleep(0.5)
                        events = p.getKeyboardEvents(cid)
                    del events[spaceKey]
                    #print(time_anim)
                elif cKey in events:
                    Camera(cid, botId, 11)
                elif qKey in events:
                    flag = True
                elif mKey in events and time.time() - m_time > 0.5:
                    m_time = time.time()
                    vis_col_state = not(vis_col_state)
                    for box in collision_view:
                        colour = list(p.getVisualShapeData(box)[0][7])
                        if vis_col_state:
                            colour[3] = 0.2
                        else:
                            colour[3] = 0.0
                        p.changeVisualShape(box, -1, rgbaColor=colour)
                events = p.getKeyboardEvents(cid)
            if flag:
                events.update({qKey : 1})

            if xKey in events:
                break
        if loop == False:
            break

    del events[qKey]


    while qKey not in events and xKey not in events:
        events = p.getKeyboardEvents(cid)
        time.sleep(0.5)

    Disconnect()


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

def view_trajectory(trajectories, goals=None, success_list=None, visual_fn = lambda q: False, interpolate_gaps=False, loop=True):
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
            #random_brick_gen(cid)

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
                    current_pose = p.getLinkState(botId, config.EEF_ID)[0]
                    p.addUserDebugLine(lineFromXYZ=config.prev_pose, lineToXYZ=current_pose, physicsClientId=cid, lifeTime=config.LINE_LIFE, lineColorRGB=[1,0,0], lineWidth=2)
                    config.prev_pose = current_pose

                time.sleep(0.1)
                last_t_id = current_t_id


                events = p.getKeyboardEvents(cid)
                if ord('p') in events:
                    print("Plotting graph")
                    graph_trajectory(trajectory)

    Disconnect()

def graph_path(path):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()

    for joint in np.transpose(path):
        t = list(range(len(joint)))
        ax.plot(t, joint)

    plt.show()

def graph_trajectory(trajectory):
    #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    path = []
    timings = []

    for k in trajectory.keys():
        path.append(trajectory[k]['p'])
        timings.append(trajectory[k]['t'])

    import matplotlib.pyplot as plt
    from numpy.polynomial import polynomial as poly
    fig, ax = plt.subplots()

    for count, joint in enumerate(np.transpose(path)):
        line_normal, = ax.plot(timings, joint, '-x')
        line_normal.set_label(config.LINK_ID_LIST[count+2])
        fitted_poly = poly.polyfit(timings, joint, deg=3)
        fitted_vals = poly.polyval(timings, fitted_poly)
        line_fitted, = ax.plot(timings, fitted_vals, 'o', color=plt.gca().lines[-1].get_color())
        line_fitted.set_label(config.LINK_ID_LIST[count+2] + "_cubic")

    ax.legend()
    plt.show()
    #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)