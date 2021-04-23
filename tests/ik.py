# Can only run where ikfast_pybind has been built
# TODO (swilcock0) : Add unittests to assert similarity
# TODO (swilcock0) : Add comments

from vs068_pb.ik_fk import getIK_FN
from vs068_pb.config import set_IKFAST, Pose
from vs068_pb.utils import Disconnect, check_same, quick_load_bot

Disconnect()
pose_test = Pose([0,0,1], [1,0,0,0])
botId, cid = quick_load_bot()


def testIK(pose):
    set_IKFAST(True)
    ik = getIK_FN()
    test1_a = ik(pose[0], pose[1])
    
    set_IKFAST(False)
    ik = getIK_FN()
    test1_b = ik(pose[0], pose[1], cid=cid, botId=botId)
    print(test1_a)
    print(test1_b)

testIK(pose_test)







