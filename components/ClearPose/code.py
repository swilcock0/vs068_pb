"""
Resets the stickied EEF pose and recomputes the solution
    Args:
        on: BOOL
"""

__author__ = "Sam"
__version__ = "2021.05.12"

import rhinoscriptsyntax as rs
import scriptcontext as sc
import Grasshopper
from Grasshopper.Kernel import GH_Param, GH_Convert, Parameters

ghenv.Component.Message = 'ClearPose v0.1'

def ghSolutionRecompute():
    """ Recomputes the Grasshopper solution (ala pressing F5) """
    
    def expireAllComponentsButThis(e):
        for obj in ghenv.Component.OnPingDocument().Objects:
            if not obj.InstanceGuid == ghenv.Component.InstanceGuid:
                obj.ExpireSolution(False)
                
    ghenv.Component.OnPingDocument().ScheduleSolution(1000,expireAllComponentsButThis)



if on:
    sc.sticky["pose"] = [0,0,0]
    ghSolutionRecompute()