# Can only run where ikfast_pybind has been built
# TODO (swilcock0) : Add unittests to assert similarity
# TODO (swilcock0) : Add comments

from vs068_pb.ik_fk import getIK_FN, getFK_FN, sample_ik, prune_invalid_poses, get_valid_ik
from vs068_pb.config import set_IKFAST
import vs068_pb.config as config
from vs068_pb.utils import Disconnect, check_same, quick_load_bot, set_joint_states, quat_from_euler, save_state, restore_state, Pose, loadFloor, Camera
from pybullet import GUI
import pybullet as p
import time
from math import degrees, radians

Disconnect()

botId, cid = quick_load_bot()
state_init = save_state(cid)
solutions = []

def append_if_non_empty(append_to, app):
    if not (app == []):
        append_to.append(app)

    return append_to

append_if_non_empty(
    solutions,
    get_valid_ik(pose=Pose([0,0,0.9], [radians(180),radians(0),radians(0)]), cid=cid, botId=botId, validate=False, attempts=10000)
    )

append_if_non_empty(
    solutions,
    get_valid_ik(pose=Pose([0,0,0.9], [radians(180),radians(0),radians(0)]), cid=cid, botId=botId, validate=False, attempts=10000)
    )

append_if_non_empty(
    solutions,
    get_valid_ik(pose=Pose([0,0,1.4], [radians(180),radians(0),radians(-90)]), cid=cid, botId=botId, validate=True, attempts=10000, num_candidates=100)
    )

append_if_non_empty(
    solutions,
    get_valid_ik(pose=Pose([0.3,0.3,0.3], [radians(0), radians(0),radians(0)]), cid=cid, botId=botId, validate=True, attempts=10000, num_candidates=100)
    )


append_if_non_empty(
    solutions,
    get_valid_ik(pose=Pose([0.3,0.3,1.0], [0,0,0]), cid=cid, botId=botId, validate=True, attempts=10000)
    )

config.ANGL_TOL = radians(359)
append_if_non_empty(
    # TODO : Double check the joint limits
    solutions,
    get_valid_ik(pose=Pose([0.8,0,0.8], [radians(90), radians(90),radians(90)]), cid=cid, botId=botId, validate=False, num_candidates=100)
    )

append_if_non_empty(
    # TODO : Double check the joint limits
    solutions,
    get_valid_ik(pose=Pose([00,0,1.0], [0,0,0]), cid=cid, botId=botId, validate=True, attempts=10000)
    )

Disconnect()

botId, cid = quick_load_bot(GUI)
loadFloor(cid)

for a in range(10):
    for j in solutions:
        Camera(cid, botId, 11)
        for i in range(len(j)):
            conf = j[i]
            #print(solutions[i])
            set_joint_states(cid, botId, config.info.free_joints, conf, [0]*6)
            #time.sleep(1)        
            Camera(cid, botId, 11)
Disconnect()






