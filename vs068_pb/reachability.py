import numpy as np
import pickle
import os
import time
import datetime
#import pybullet as p
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from vs068_pb.ik_fk import get_valid_ik
from vs068_pb.utils import quick_load_bot, Disconnect, get_link_pose
import vs068_pb.config as config


class Reachability(object):
    def __init__(self):
        src_fldr = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources")
        self.reach_file = os.path.join(src_fldr, "Reachmap")
        
        
    def generate_map(self, collisions=True):
        print("Building reachability map...")
        config.DEBUG = False
        botId, cid = quick_load_bot()
        workspace_min = -0.85
        workspace_max = 0.85
        z_max = 1.42
        discretisation = 50*(1e-3) # 5 cm
        self.data = []
        numVoxels = 0
        gm_start = time.time()
    
        N = int((workspace_max - workspace_min)/discretisation)
        N_z = int((z_max+0.5)/discretisation)
        #spacing = ((workspace_max-workspace_min) * 1.0 / np.floor(np.sqrt(N)))

        for x in np.linspace(workspace_min, workspace_max, N):
            for y in np.linspace(workspace_min, workspace_max, N):
                for z in np.linspace(-0.5, z_max, N_z):
                    numVoxels += 1
                    successes = self.newVoxel(x_c=x, y_c=y, z_c=z, r=discretisation/2, N=16, collisions=collisions, cid=cid, botId = botId)
                    if (successes > 0):
                        self.data.append([x, y, z, successes/numVoxels]) 
                        print(str(self.data[-1]) + ", Num poses tested: " + str(numVoxels))
                    
                    if numVoxels % 10 == 0:
                        pc_done = numVoxels/(N*N*N_z)
                        elapsed_time = (time.time()-gm_start)
                        projected_time = elapsed_time/max(pc_done, 0.0001)
                        remaining_time = projected_time - elapsed_time

                        elapsed_time_str = str(datetime.timedelta(seconds=int(elapsed_time)))
                        projected_time_str = str(datetime.timedelta(seconds=int(projected_time)))
                        remaining_time_str = str(datetime.timedelta(seconds=int(remaining_time)))

                        print("{:.2%} done".format(pc_done))
                        print("{} / {}. {} remaining".format(elapsed_time_str, projected_time_str, remaining_time_str))
        print
        Disconnect()
        
        
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

    def view_data(self):
        cmap = LinearSegmentedColormap.from_list('my_rbg', 
            [(1.0, 0.0, 0.0),
            (0.5, 0.0, 0.5),
            (0.0, 0.0, 1.0),
            (0.0, 0.5, 0.5),
            (0.0, 1.0, 0.0)])
        x = [d[0] for d in self.data]
        y = [d[1] for d in self.data]
        z = [d[2] for d in self.data]
        c = [d[3] for d in self.data]

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(x, y, z, c=c, cmap=cmap)
        ax.set_title('Reachability')

        ax.set_xlabel("X")
        ax.set_ylabel("Y")


        plt.show()

    def newVoxel(self, x_c=0.0, y_c=0.0, z_c=0.0, r=1.0, N=16, collisions=True, cid=0, botId = 0):
        ''' Define a spherical voxel and test the poses around it ''' 
        num_successes = 0
        
        n = int(np.floor(np.sqrt(N)))

        # Set points distributed evenly. Could change to generalised spiral as J. Saff and A. Kuijlaars, “Distributing many points on a sphere"
        self.testPose(0, 0, r, x_c, y_c, z_c, collisions=collisions, cid=cid, botId = botId)
        self.testPose(np.pi, np.pi, r, x_c, y_c, z_c, collisions=collisions, cid=cid, botId = botId)
        
        for phi in np.linspace(0, 2*np.pi, n, 1):
            for theta in np.linspace(0, np.pi, n, 0 ):
                if self.testPose(phi, theta, r, x_c, y_c, z_c, collisions=collisions, cid=cid, botId = botId):
                    num_successes += 1
        return num_successes

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
        pose =[[x,y,z], [qx, qy, qz, qw]]

        #self.poses[self.numPoses] = {self.numPoses: {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}}}
        #self.numPoses += 1
        
        if collisions:
            confs = get_valid_ik(pose, cid=cid, botId=botId, validate=True, attempts=100,  epsilon=0.001,  angle=0.01)
        else:
            confs = get_valid_ik(pose, cid=cid, botId=botId, validate=False, attempts=100,  epsilon=0.001,  angle=0.01)
        
        success = len(confs) > 0
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

if __name__ == "__main__":
    preload = False
    Disconnect()
    test = Reachability()

    if preload:
        test.load_data()  
    else:
        test.generate_map(collisions=True)
        test.dump_data()
  
    data = sorted(test.data, key=lambda x: x[3])
    for datum in data:
        print(datum)
    
    test.view_data()