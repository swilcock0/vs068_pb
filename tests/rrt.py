import cProfile, pstats, io
from vs068_pb.rrt import rrt
from pstats import SortKey
from vs068_pb.utils import Disconnect, quick_load_bot, set_joint_states
from vs068_pb.ik_fk import getFK_FN
import pybullet as p
import vs068_pb.config as config
import time

goal =  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


#pr = cProfile.Profile()
#pr.enable()
path = rrt((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), goal, n_it = 1000, time_limit = 10.0, visualise=-1, tool_space=True, tolerance=[0.01, 0.1])
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
for j in range(12):
    p.changeVisualShape(botId, j, rgbaColor=[0.0, 1.0, 0.0, 0.3])
set_joint_states(cid, botId, config.info.free_joints, goal, [0]*6)

botId, cid = quick_load_bot(p.GUI, physicsClient=cid)
#loadFloor(cid)
time_anim = 5.0

events = p.getKeyboardEvents(cid)
qKey = ord('q')
xKey = ord('x')

while qKey not in events and xKey not in events:
    for q in path:
        set_joint_states(cid, botId, config.info.free_joints, q, [0]*6)
        time.sleep(time_anim/len(path))      
        events = p.getKeyboardEvents(cid)
        if xKey in events:
            events[qKey] = 1 # TODO (swilcock0) : Fix this test
            break    

del events[qKey]


while qKey not in events and xKey not in events:
    events = p.getKeyboardEvents(cid)
    time.sleep(0.5)



Disconnect()
