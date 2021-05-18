from vs068_pb.utils import quat_from_euler, Disconnect
from vs068_pb.planning_scene import Geometry
import vs068_pb.config as config
import os
import pybullet as p
from math import radians

Disconnect()
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
geoms = []

def test_boxes():
    box = Geometry(physicsClientId=physicsClient)
    box.define_box()

    box.add_visual()
    box.add_collision()
    geoms.append(box)

    box2 = Geometry(physicsClientId=physicsClient)
    box2.define_box(pose_centre=[[2,0,1], list(quat_from_euler([radians(0), radians(45), radians(30)]))], dimensions=[1,1,2])

    box2.add_visual()
    box2.add_collision()
    geoms.append(box2)
    input()
    box.clear()
    box2.clear()

def test_spheres():
    sphere = Geometry(physicsClientId=physicsClient)
    sphere.define_sphere()
    
    sphere.add_visual()
    sphere.add_collision()
    geoms.append(sphere)

    sphere2 = Geometry(physicsClientId=physicsClient)
    sphere2.define_sphere(pose_centre=[[2,0,1], [0,0,0,1]], radius=1)

    sphere2.add_visual()
    sphere2.add_collision()
    geoms.append(sphere2)
    input()
    sphere.clear()
    sphere2.clear()

def test_planes():
    plane = Geometry(physicsClientId=physicsClient)
    plane.define_plane()
    
    plane.add_visual()
    plane.add_collision()
    geoms.append(plane)

    plane2 = Geometry(physicsClientId=physicsClient)
    plane2.define_plane(pose_centre=[[2,0,1], list(quat_from_euler([radians(0), radians(45), radians(30)]))])

    plane2.add_visual()
    plane2.add_collision()
    geoms.append(plane2)
    input()
    plane.clear()
    plane2.clear()

def test_meshes():
    brick_file = os.path.join(config.src_fldr, "meshes", "BrickVisual.stl")
    mesh = Geometry(physicsClientId=physicsClient, safety_margin=0.2, colour=[0.8, 0.25, 0.33, 1.0] )
    mesh.define_mesh(brick_file, scale=1/1000)
    
    mesh.add_visual()
    mesh.add_collision()
    geoms.append(mesh)

    mesh2 = Geometry(physicsClientId=physicsClient, safety_margin=0.2, colour=[0.8, 0.25, 0.33, 1.0] )
    mesh2.define_mesh(brick_file, scale=1/1000, pose_centre=[[2,0,1], list(quat_from_euler([radians(0), radians(45), radians(30)]))])

    mesh2.add_visual()
    mesh2.add_collision()
    geoms.append(mesh2)

    input()
    mesh.hide_collision_visual()
    input()
    mesh.hide_visual()
    input()
    mesh.show_collision_visual()
    input()
    mesh.show_visual()
    input()
    mesh2.clear()

test_boxes()
test_spheres()
test_planes()
test_meshes()

for g in geoms:
    print(g)