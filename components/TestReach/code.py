"""
On boolean true input, load the reach map data
    Args:
        on: BOOL
    Returns:
        data: The loaded data.
        X : X data
        Y : Y data
        Z : Z data
        R : Reach data
"""

ghenv.Component.Message = 'TestReach v1.0'

import scriptcontext as sc
import rhinoscriptsyntax as rs
import Rhino as r
import Rhino.Geometry as rg
import System.Drawing as sd

data = sc.sticky.get("data", None)

if on:
    from compas.rpc import Proxy

    # pythonw prevents a window from opening
    with Proxy('vs068_pb.ghpython', python='C:\Users\Sam\Anaconda3\envs\project\pythonw.exe') as proxy:
        data = proxy.load_reach_data()
        sc.sticky["data"] = data
        
def column(A, i):
    return [row[int(i)] for row in A]

X = column(data, 0)
Y = column(data, 1)
Z = column(data, 2)
R = column(data, 3)

#pnts = [[rs.AddPoint([X[i], Y[i], Z[i]])] for i in range(len(X))]
pnts = []
for i in range(len(X)):
    pnts.append(rs.AddPoint([X[int(i)], Y[int(i)], Z[int(i)]]))
    
reach = R


display = r.Display.CustomDisplay(True)
for i in range(len(X)):
    x = X[i]
    y = Y[i]
    z = Z[i]
    c = R[i]
    
    r_c = max(0,int(255*(0.5-c)/0.5))
    g_c = max(0,int(255*(c-0.5)/0.5))
    b_c = int(255*abs(0.5-abs(0.5-c))/0.5)*c


    color = sd.Color.FromArgb(r_c, g_c, b_c)
    Pt = rg.Point3d(100*x,100*y,100*z)
    display.AddPoint(Pt, color, r.Display.PointStyle.X, 2)