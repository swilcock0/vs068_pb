import cProfile, pstats, io
from vs068_pb.rrt import rrt
from pstats import SortKey
from vs068_pb.utils import Disconnect, quick_load_bot, set_joint_states, Camera
from vs068_pb.ik_fk import getFK_FN
import pybullet as p
import vs068_pb.config as config
import time
import math

goals =  [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), \
        (1.0, 1.0, 1.0, 1.0, 1.0, 1.0), (-1.0, -1.0, -1.0, -1.0, -1.0, -1.0), (3, -0.2, -0.5, -1.0, -1.0, 2.0),\
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
#print(config.screen_width)

#pr = cProfile.Profile()
#pr.enable()
#path = rrt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=-1, tool_space=True, tolerance=[0.01, 0.1])
path = []

for i in range(1, len(goals)):
    path_section = rrt(goals[i-1], goals[i], n_it = 1000, time_limit = 10.0, tool_space=False, tolerance=0.1)
    path = path + path_section

#pr.disable()
#print(path)
#s = io.StringIO()
#sortby = SortKey.CUMULATIVE
#ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
#ps.print_stats()
#print(s.getvalue())

# Show on robot
Disconnect()

botId, cid = quick_load_bot(p.GUI)
p.changeVisualShape(botId, 0, rgbaColor=[0.0, 1.0, 0.0, 0.0])
for j in range(1, 12):
    p.changeVisualShape(botId, j, rgbaColor=[0.0, 1.0, 0.0, 0.3])
set_joint_states(cid, botId, config.info.free_joints, goals[1], [0]*6)

botId, cid = quick_load_bot(p.GUI, physicsClient=cid)
#loadFloor(cid)
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
    for q in path:
        if time.time() - start_time > 1.3*(time_anim/len(path)):
            # Speed up lagging depiction
            #print(time.time() - start_time)
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
            print(time_anim)
        elif downKey in events:
            time_anim += 1
            #print(time_anim)
        elif spaceKey in events and time.time() - space_time > 0.2:
            space_time = time.time()
            tmp = time_anim_bak
            time_anim_bak = time_anim
            time_anim = tmp
            #print(time_anim)
        elif cKey in events:
            Camera(cid, botId, 11)
        elif qKey in events:
            flag = True
    if flag:
        events.update({qKey : 1})

del events[qKey]


while qKey not in events and xKey not in events:
    events = p.getKeyboardEvents(cid)
    time.sleep(0.5)



Disconnect()
