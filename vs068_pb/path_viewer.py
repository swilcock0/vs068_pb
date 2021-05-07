import time
import pybullet as p
from vs068_pb.utils import set_joint_states, loadFloor, Disconnect, quick_load_bot, Camera
import vs068_pb.config as config
import math

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
    loadFloor(cid)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Set up loop vars
    events = p.getKeyboardEvents(cid)
    time_anim = 5.0
    qKey = ord('q')
    xKey = ord('x')
    cKey = ord('c')
    mKey = ord('m')
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
                if time.time() - start_time > 1.3*(time_anim/len(path)):
                    # Speed up lagging depiction
                    #print(time.time() - start_time)
                    #print(num_delays)
                    if num_delays == 0:
                        num_delays = int((time.time()-start_time)/(time_anim/len(path)))
                        start_time = time.time()
                    else:
                        num_delays -= 1
                    
                    continue

                start_time = time.time() 
                if time_anim < 100:
                    set_joint_states(cid, botId, config.info.free_joints, q, [0]*6)
                time.sleep(time_anim/len(path))  

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