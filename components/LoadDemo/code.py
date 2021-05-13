"""
Loads the arm demo example. On exit, returns the final position of the EEF.
    Args:
        on: BOOL
    Returns:
        pose: Final EEF pose
"""
        
ghenv.Component.Message = 'LoadDemo v0.1'
        
import rhinoscriptsyntax as rs
import time
import scriptcontext as sc

pose = sc.sticky.get("pose", [0,0,0])
        
if on:
    from compas.rpc import Proxy
        
    # pythonw prevents a window from opening
    with Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\python.exe') as proxy:
        pose = proxy.load_demo()
        sc.sticky["pose"] = pose