''' Test IK Generator '''

from vs068_pb.utils import quick_load_bot, Disconnect
from vs068_pb.ik_fk import get_ik_generator, getIK_FN
from vs068_pb.config import Pose
import pybullet as p

po = Pose([0.0,0.0,1.0], [1.0,0.0,0.0,0.0])

Disconnect()
botId, _ = quick_load_bot()

gen = get_ik_generator(0, po)

next(gen)
next(gen)