# Can only run where ikfast_pybind has been built
# TODO (swilcock0) : Add unittests to assert similarity
# TODO (swilcock0) : Add comments

from vs068_pb.ik_fk import getIK_FN, getFK_FN, sample_ik, prune_invalid_poses
from vs068_pb.config import set_IKFAST
import vs068_pb.config as config
from vs068_pb.utils import Disconnect, check_same, quick_load_bot, set_joint_states, quat_from_euler, save_state, restore_state, Pose
from pybullet import GUI
import time
from math import degrees, radians

Disconnect()

botId, cid = quick_load_bot()


def testIK(pose, valid_only=True):
    set_IKFAST(True)
    ik = getIK_FN()
    solutions = sample_ik(cid, botId, pose)

    if valid_only:
        solutions = prune_invalid_poses(solutions, cid=cid, botId=botId)

    for i in range(min(len(solutions), 10)):
        set_joint_states(cid, botId, config.info.free_joints, solutions[2*i], [0]*6)
        time.sleep(0.5)


state_init = save_state(cid)

testIK(Pose([0,0,0.9], [radians(180),radians(0),radians(0)]))
testIK(Pose([0,0,1.4], [radians(180),radians(0),radians(0)]))
testIK(Pose([0.3,0.3,0.3], [radians(90), radians(90),radians(90)]))
testIK(Pose(point=[0.3,0.3,1.0], euler=[0,0,0]), True)
testIK(Pose(point=[00,0,1.0], euler=[0,0,0]), True)

restore_state(state_init, cid)
time.sleep(5.0)
Disconnect()






