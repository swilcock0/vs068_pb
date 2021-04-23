# Can only run where ikfast_pybind has been built
# TODO (swilcock0) : Add unittests to assert similarity
# TODO (swilcock0) : Add comments

from vs068_pb.ik_fk import getFK_FN
from vs068_pb.config import set_IKFAST
import pybullet as p
import os
from vs068_pb.utils import Disconnect
from numpy import array, concatenate

Disconnect()
src_fldr = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources")
urdf = os.path.join(os.path.dirname(os.path.realpath(__file__)),  '..', "resources", "vs068_with_gripper_pybullet.urdf")  

cid = p.connect(p.DIRECT)
p.setAdditionalSearchPath(src_fldr)
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])

# Load the robot
botId = p.loadURDF(urdf, startPos, startOrientation, useFixedBase=1)


def testIK(joint_positions, tol=0.01):
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


def check_same(a, b, tol):
    #print(a)
    #print(b)
    flt_a = concatenate(array(a, dtype=object).reshape(-1))
    flt_b = concatenate(array(b, dtype=object).reshape(-1))
    
    same = True
    
    for i in range(len(flt_a)):
        if (flt_a[i] > (flt_b[i] + tol)) or (flt_a[i] < (flt_b[i] - tol)):
            same = False
            break
    return same


tolerance_dict = {}  
for i in range(10):
    tol = 1./(10**i)
    a = testIK([0,0,0,0,0,0], tol)
    b = testIK([1,1,1,1,1,1], tol)
    
    tolerance_dict[tol] = a and b
    
print(tolerance_dict)











