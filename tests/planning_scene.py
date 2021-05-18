from vs068_pb.planning_scene import Scene, Geometry
from vs068_pb.utils import quick_load_bot, Disconnect

import pybullet as p


botId, cid = quick_load_bot(p.GUI)

MyScene = Scene(cid, botId)


## Boxes
dims = [[0.2, 0.2, 0.2], [0.1, 0.1, 1.2], [0.1, 0.1, 1.2]]
pos = [[0.3,0.3,0.7], [-0.3, -0.3, 0.6], [-0.35, 0.35, 0.6]]


for i in range(len(dims)):
    box = Geometry(physicsClientId=cid)

    box.define_box([pos[i], [0,0,0,1]], dims[i])

    MyScene.add_object(box)


input("Added boxes... [PRESS ENTER]")


rads = [0.1, 0.05, 0.05]
pos = [[0.3,0.3,0.8], [-0.3, -0.3, 1.2], [-0.35, 0.35, 1.2]]
for i in range(len(rads)):
    sphere = Geometry(physicsClientId=cid)

    sphere.define_sphere([pos[i], [0,0,0,1]], radius=rads[i])

    MyScene.add_object(sphere)

input("Added spheres... [PRESS ENTER]")
for g in MyScene.collision_objects:
    print(MyScene.collision_objects[g])

MyScene.initialise_allowed()
print("Collision: {}".format(MyScene.check_collisions([1.0,1.0,1.0,1.0,1.0,1.0])))

input("Tested collisions... [PRESS ENTER")

Disconnect()
botId, cid = quick_load_bot(p.GUI)
input("Started new physics client... [PRESS ENTER]")

MyScene.change_physics_clients(cid)
input("Tested re-adding scene... [PRESS ENTER]")