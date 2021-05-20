"""
On boolean true input, load a planning scene from mesh inputs
    Args:
        kill: Kill the proxy server and delete proxy sticky
"""

__author__ = "Sam"
__version__ = "2021.05.20"
ghenv.Component.Message = 'KillScene v0.1'

import rhinoscriptsyntax as rs
import scriptcontext as sc

st = sc.sticky

if kill:
    if st["proxy_scene"]:
        st["proxy_scene"].stop_server()
        st["proxy_scene"] = None