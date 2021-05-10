import numpy as np
import pickle
import os
import time
import datetime
import pybullet as p
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import plotly.io as io
from math import radians

from vs068_pb.ik_fk import get_valid_ik
from vs068_pb.utils import quick_load_bot, Disconnect, get_link_pose, Pose, set_joint_positions#, PlotlyViewer
import vs068_pb.config as config


class Reachability(object):
    def __init__(self):
        src_fldr = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources", "pickles")
        self.reach_file = os.path.join(src_fldr, "Reachmap")
        self.reach_file_collisions = os.path.join(src_fldr, "ReachmapCollisions")
        
        
    def generate_map(self, collisions=True):
        print("Building reachability map...")
        config.DEBUG = False
        botId, cid = quick_load_bot()
        workspace_min = -0.95
        workspace_max = 0.95
        z_min = -0.4   
        z_max = 1.5
        discretisation = 50*(1e-3) # 5 cm
        self.data = []
        self.verbose_data = {}
        numVoxels = 0.0
        gm_start = time.time()
        sec_timer = time.time()
        numPoses = 0.0
        
        N = int((workspace_max - workspace_min)/discretisation)
        N_z = int((z_max-z_min)/discretisation)
        #spacing = ((workspace_max-workspace_min) * 1.0 / np.floor(np.sqrt(N)))

        for x in np.linspace(workspace_min, workspace_max, N):
            for y in np.linspace(workspace_min, workspace_max, N):
                for z in np.linspace(z_min, z_max, N_z):
                    numVoxels += 1
                    successes, num_poses = self.newVoxel(x_c=x, y_c=y, z_c=z, r=discretisation/2, N=25, collisions=collisions, cid=cid, botId = botId)
                    numPoses += num_poses

                    if (successes > 0):
                        self.data.append([x, y, z, successes/num_poses]) 
                        if numPoses % 1000 == 0:
                            print(str(self.data[-1]) + ", Num poses tested: " + str(numPoses))
                    
                    if time.time() - sec_timer >= 1.0:
                        pc_done = numVoxels/(N*N*N_z)
                        elapsed_time = (time.time()-gm_start)
                        projected_time = elapsed_time/max(pc_done, 0.0001)
                        remaining_time = projected_time - elapsed_time

                        elapsed_time_str = str(datetime.timedelta(seconds=int(elapsed_time)))
                        projected_time_str = str(datetime.timedelta(seconds=int(projected_time)))
                        remaining_time_str = str(datetime.timedelta(seconds=int(remaining_time)))

                        print("{:.2%} done".format(pc_done))
                        print("{} / {}. {} remaining".format(elapsed_time_str, projected_time_str, remaining_time_str))
                        sec_timer = time.time()
        print("Total {} voxels, {} poses tested in {}. ".format(numVoxels, numPoses, elapsed_time_str))
        Disconnect()

    def newVoxel(self, x_c=0.0, y_c=0.0, z_c=0.0, r=1.0, N=16, collisions=True, cid=0, botId = 0):
        ''' Define a spherical voxel and test the poses around it ''' 
        num_successes = 0
        num_poses = 0
        n = int(np.floor(np.sqrt(N)))

        # Set points distributed evenly. Could change to generalised spiral as J. Saff and A. Kuijlaars, “Distributing many points on a sphere"
        self.testPose(0, 0, r, x_c, y_c, z_c, collisions=collisions, cid=cid, botId = botId)
        self.testPose(np.pi, np.pi, r, x_c, y_c, z_c, collisions=collisions, cid=cid, botId = botId)
        
        for phi in np.linspace(0, 2*np.pi, n, 1):
            for theta in np.linspace(0, np.pi, n, 0 ):
                num_poses += 1
                if self.testPose(phi, theta, r, x_c, y_c, z_c, collisions=collisions, cid=cid, botId = botId):
                    num_successes += 1
        return num_successes, num_poses

    def testPose(self, phi, theta, r=1.0, x_c=0.0, y_c=0.0, z_c=0.0, collisions=True, cid=0, botId = 0):
        ''' Test a pose in spherical coordinate definition '''
        st = np.sin(theta)
        x = r*np.cos(phi)*st + x_c
        y = r*np.sin(phi)*st + y_c
        z = r*np.cos(theta) + z_c
        
        roll = 0
        pitch = theta+np.pi
        yaw = phi
        
        qx, qy, qz, qw = self.eulerToQuaternion(roll, pitch, yaw)
        pose = [[x,y,z], [qx, qy, qz, qw]]

        #self.poses[self.numPoses] = {self.numPoses: {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}}}
        #self.numPoses += 1
        
        if collisions:
            confs = get_valid_ik(pose, cid=cid, botId=botId, validate=True, attempts=10,  epsilon=0.02,  angle=radians(1), get_one=True)
        else:
            confs = get_valid_ik(pose, cid=cid, botId=botId, validate_collisions=False, attempts=10,  epsilon=0.02,  angle=radians(1), get_one=True)
        
        success = len(confs) > 0
        #self.verbose_data[len(self.verbose_data)] = {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}, 'success': success}

        return success


    def eulerToQuaternion(self, x, y, z):
        ''' Convert Euler/RPY to quaternion '''
        cx = np.cos(x * 0.5)
        sx = np.sin(x * 0.5)
        cy = np.cos(y * 0.5)
        sy = np.sin(y * 0.5)
        cz = np.cos(z * 0.5)
        sz = np.sin(z * 0.5)

        qx = (sx * cy * cz) - (cx * sy * sz)
        qy = (cx * sy * cz) + (sx * cy * sz)
        qz = (cx * cy * sz) - (sx * sy * cz)
        qw = (cx * cy * cz) + (sx * sy * cz)

        return qx, qy, qz, qw   

    def dump_data(self, file=None, data=None):
        if file == None:
            file = self.reach_file
        if data == None:
            data = self.data

        with open(file + '.pickle', 'wb') as handle:
            pickle.dump(data, handle, protocol=4)
        with open(file + '2.pickle', 'wb') as handle:
            pickle.dump(data, handle, protocol=2)
        print("Data dumped!")

    def load_data(self, file=None):
        if file == None:
            file = self.reach_file
        
        with open(file + '.pickle', mode='rb') as handle:
            self.data = pickle.load(handle)

    def view_data(self, every=1, output_html=False):
        x = [d[0] for d in self.data]
        y = [d[1] for d in self.data]
        z = [d[2] for d in self.data]
        c = [d[3] for d in self.data]

        if every != 1:
            x = x[::int(every)]
            y = y[::int(every)]
            z = z[::int(every)]
            c = c[::int(every)]

        if config.PLOTLY_AVAILABLE:
            df = pd.DataFrame({'X' : x, 'Y' : y, 'Z' : z, 'Reachability' : c})

            fig = go.Figure()
            
            fig.add_trace(
                go.Scatter3d(
                    x=df[df['Y']<0]['X'], y=df[df['Y']<0]['Y'], z=df[df['Y']<0]['Z'], 
                    mode ='markers', 
                    marker=dict(
                        color=df[df['Y']<0]['Reachability'], 
                        colorscale='RdYlGn',
                        colorbar=dict(
                            x=1.02,
                            title="Reachability"
                        )
                    ),
                    visible=False
                )
            )
            fig.add_trace(
                go.Scatter3d(
                    x=df['X'], y=df['Y'], z=df['Z'],
                    mode ='markers', 
                    marker=dict(
                        color=df['Reachability'], 
                        colorscale='RdYlGn',
                        colorbar=dict(
                            x=1.02,
                            title="Reachability"
                        )
                    )
                )
            )
            fig.add_trace(
                go.Scatter3d(
                    x=df[df['Z']<0.8]['X'], y=df[df['Z']<0.8]['Y'], z=df[df['Z']<0.8]['Z'],
                    mode ='markers', 
                    marker=dict(
                        color=df[df['Z']<0.8]['Reachability'], 
                        colorscale='RdYlGn',
                        colorbar=dict(
                            x=1.02,
                            title="Reachability"
                        )
                    ),
                    visible=False
                )
            )

            fig.update_layout(
                updatemenus=[
                    dict(
                        type = "buttons",
                        direction = "left",
                        buttons=list([
                            dict(
                                args=[{"visible": [False, True, False],
                                    "title": "Reachability : Full"}],
                                label="Full",
                                method="update"
                            ),
                            dict(
                                args=[{"visible": [True, False, False],
                                    "title": "Reachability : Half"}],
                                label="Half",
                                method="update"
                            ),
                            dict(
                                args=[{"visible": [False, False, True],
                                    "title": "Reachability : Top"}],
                                label="Bottom",
                                method="update"
                            )
                        ]),
                        pad={"r": 10, "t": 10},
                        showactive=True,
                        x=0.11,
                        xanchor="left",
                        y=1.1,
                        yanchor="top",
                    ),
                ],
                title="Reachability Map",
                title_x=0.5
            )

            fig.show(renderer="browser")#pv = PlotlyViewer(fig)

            if output_html:
                io.write_html(fig, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources", "Reach.html"), include_plotlyjs='cdn', auto_open=True)
        else:
            fig = plt.figure()
            ax = plt.axes(projection='3d')
            ax.scatter3D(x, y, z, c=c, cmap=cmap)
            ax.set_title('Reachability')

            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlim(-0.5, 1.5)
            plt.show()



if __name__ == "__main__":
    preload = True
    Disconnect()
    test = Reachability()

    if preload:
        test.load_data(file=test.reach_file_collisions)  
    else:
        test.generate_map(collisions=False)
        test.dump_data(file=test.reach_file)

        test.generate_map(collisions=True)
        test.dump_data(file=test.reach_file_collisions)
        
        # For overnight run, hibernate after
        #os.system("Rundll32.exe Powrprof.dll,SetSuspendState Sleep")

    test.view_data(every=1)
    
