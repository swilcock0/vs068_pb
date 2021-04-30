''' SAMPLER NOT WORKING YET '''
from vs068_pb.utils import quick_load_bot, Disconnect
from vs068_pb.ik_fk import sample_tool_ik
from vs068_pb.config import Pose

po = Pose([0,0,1.4], [1,0,0,0])

Disconnect()
botId, _ = quick_load_bot()


# Initial tests without IKFAST
# TODO (swilcock0) : Build an IK test


print("")
print("")
print("")
print("")


sample_tool_ik(botId, po)


print("")
print("")
print("")
print("")

