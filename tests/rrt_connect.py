import cProfile, pstats, io
from vs068_pb.rrt_connect import rrt_connect
from pstats import SortKey
from vs068_pb.utils import Disconnect, quick_load_bot, set_joint_states, Camera, loadFloor, interval_generator, create_box_collisions, Pose,\
    size_all
import pybullet as p
import vs068_pb.config as config
import time
import math
import sys

dims = [[0.2, 0.2, 0.2], [0.1, 0.1, 1.2], [0.1, 0.1, 1.2]]
pos = [[0.3,0.3,0.7], [-0.3, -0.3, 0.6], [-0.35, 0.35, 0.6]]
collision_fn, visual_fn = create_box_collisions(dims, pos)

goals =  [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

generator = interval_generator(config.lower_lims, config.upper_lims, use_halton=True)
print("Finding valid configurations")
for i in range(100):
    if i % 10 == 0:
        print(i)
    next_goal = next(generator)

    while (collision_fn(next_goal) == True):
        next_goal = next(generator)
    goals.append(tuple(next_goal))

goals.append(goals[0])
#print(config.screen_width)


#path = rrt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=-1, tool_space=True, tolerance=[0.01, 0.1])
paths = []
success_list = []
path_section = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

tolerance = 0.01# [0.01, 0.1] 
start_time = time.time()

# pr = cProfile.Profile()
# pr.enable()
for i in range(1, len(goals)):
    #print(goals[i-1], goals[i])
    path_section, success = rrt_connect(path_section[-1], goals[i], n_it = 1000, time_limit = 3.0, tool_space=False, step=0.1, tolerance=tolerance, visualise=0, collision_fn=collision_fn)
    paths.append(path_section)
    success_list.append(success)
    print(i)

# pr.disable()
# s = io.StringIO()
# sortby = SortKey.CUMULATIVE
# ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
# ps.print_stats()
# print(s.getvalue())
print(success_list)

print("Path : {} long, Goals : {} long".format(len(paths), len(goals)))
print("Took {:.2f} s, {:%} success rate".format(time.time() - start_time, sum(success_list)/len(success_list)))
# Show on robot
Disconnect()

goalBot, cid = quick_load_bot(p.GUI, collisions=False)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
visual_fn(cid)        

p.changeVisualShape(goalBot, 0, rgbaColor=[0.0, 1.0, 0.0, 0.0])
for j in range(1, 12):
    p.changeVisualShape(goalBot, j, rgbaColor=[0.0, 1.0, 0.0, 0.3])
set_joint_states(cid, goalBot, config.info.free_joints, goals[1], [0]*6)

botId, cid = quick_load_bot(p.GUI, physicsClient=cid)
loadFloor(cid)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

time_anim = 5.0

events = p.getKeyboardEvents(cid)

qKey = ord('q')
xKey = ord('x')
upKey = p.B3G_UP_ARROW
downKey = p.B3G_DOWN_ARROW
spaceKey = p.B3G_SPACE
start_time = time.time()
flag = False
time_anim_bak = 101.0
space_time = time.time()
cKey = ord('c')

num_delays = 0

while qKey not in events and xKey not in events:
    for count, path in enumerate(paths):
        set_joint_states(cid, goalBot, config.info.free_joints, goals[count+1], [0]*6)

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
                print(time_anim)
            elif cKey in events:
                Camera(cid, botId, 11)
            elif qKey in events:
                flag = True

            events = p.getKeyboardEvents(cid)
        if flag:
            events.update({qKey : 1})

        if qKey in events or xKey in events:
            break

del events[qKey]


while qKey not in events and xKey not in events:
    events = p.getKeyboardEvents(cid)
    time.sleep(0.5)



Disconnect()
