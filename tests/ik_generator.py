''' Test IK Generator '''

from vs068_pb.utils import quick_load_bot, Disconnect, quat_from_euler, set_joint_states
from vs068_pb.ik_fk import get_ik_generator, getIK_FN, sample_tool_ik
from vs068_pb.config import Pose
import vs068_pb.config as config
import pybullet as p
import time


po = Pose([0.0,0.0,1.0], [1,0,0,0])

Disconnect()
botId, cid = quick_load_bot(p.GUI)
time.sleep(1.0)
# print(botId, cid)

solutions = sample_tool_ik(cid, botId, po)

#print(solutions)

if solutions:
    set_joint_states(cid, botId, config.info.free_joints, solutions, [0]*6)