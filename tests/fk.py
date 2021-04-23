# Can only run where ikfast_pybind has been built
# TODO (swilcock0) : Add unittests to assert similarity
# TODO (swilcock0) : Add comments

from vs068_pb.ik_fk import getFK_FN
from vs068_pb.config import set_IKFAST
from vs068_pb.utils import Disconnect, check_same, quick_load_bot

Disconnect()

botId, cid = quick_load_bot()


def testFK(joint_positions, tol=0.01):
    set_IKFAST(True)
    fk = getFK_FN()
    test1_a = fk(joint_positions)
    
    set_IKFAST(False)
    fk = getFK_FN()
    test1_b = fk(joint_positions, botId, cid)
   
    if check_same(test1_a, test1_b, tol):
        print("EEF poses are the same for test (tolerance {}) with positions {}".format(tol, joint_positions))
        return True
    else:
        print("EEF poses are NOT the same for test (tolerance {}) with positions {}".format(tol, joint_positions))
        print("")
        print(test1_a)
        print(test1_b)
        print("")
        print("")
        return False

tolerance_dict = {}  
for i in range(10):
    tol = 1./(10**i)
    a = testFK([0,0,0,0,0,0], tol)
    b = testFK([1,1,1,1,1,1], tol)
    
    tolerance_dict[tol] = a and b
    
print(tolerance_dict)











