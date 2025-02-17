"""
On boolean true input, load a planning scene from mesh inputs
    Args:
        on: Load
        meshes: Mesh inputs as list
        scale: The system scaling factor (from robot model)
        clear: Clears the planning scene
    Returns:
        collision_objects: A list of collision object IDs
"""
        
ghenv.Component.Message = 'LoadScene v0.1'
        
import rhinoscriptsyntax as rs
import time
import scriptcontext as sc
from compas_rhino.geometry import RhinoMesh
import os
from compas.rpc import Proxy

sc.sticky["proxy_scene"] = sc.sticky.get("proxy_scene", None)

#connect_exists = getattr(sc.sticky["proxy_scene"], "is_connected", None)
#print(connect_exists)

if on: 
    if sc.sticky["proxy_scene"]:
        if sc.sticky["proxy_scene"].is_connected():
            proxy = sc.sticky["proxy_scene"]
        else:
            proxy = Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\python.exe', capture_output=True)#, autoreload=False)
            proxy.load_scene()
    else:
        proxy = Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\python.exe', capture_output=True)#, autoreload=False)
        proxy.load_scene()
        
    scaling_factor = 1/scale
    scale_arr = [scaling_factor, scaling_factor, scaling_factor]
    
    tmp_obj_path = proxy.get_temp_obj_path()

    for cnt, mesh in enumerate(meshes):
        scale_arr = [scaling_factor, scaling_factor, scaling_factor]
        centre = rs.MeshVolumeCentroid(mesh)
        pnt_centre = rs.CreatePoint(centre[0], centre[1], centre[2])
        vector_reversed = rs.CreateVector(rs.CreatePoint(-centre[0], -centre[1], -centre[2]))
        rs.ScaleObject(mesh, pnt_centre, scale_arr)
        rs.MoveObject(mesh, vector_reversed)
        
        RhinoMesh.from_geometry(mesh).to_compas().to_obj(tmp_obj_path)
        proxy.load_mesh(tmp_obj_path, pos=list(centre*scaling_factor))

        #time.sleep(1.0)
        
    proxy.clean_temp()
    
    sc.sticky["proxy_scene"] = proxy

if clear:
    if sc.sticky["proxy_scene"]:
        if sc.sticky["proxy_scene"].is_connected():
            sc.sticky["proxy_scene"].clear_scene()

if sc.sticky["proxy_scene"]:
    collision_objects = sc.sticky["proxy_scene"].get_scene_objects()