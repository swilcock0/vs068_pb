import os
import tkinter as tk
from collections import namedtuple
import numpy as np
from math import radians
import pybullet as p

try:
    import ikfast_vs068 as ikv
    IKFAST_AVAILABLE = True
except ModuleNotFoundError:
    IKFAST_AVAILABLE = False

# Get screen dimensions
root = tk.Tk()
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Data folders and locations
src_fldr = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', "resources")
urdf = os.path.join(src_fldr, "vs068_with_gripper_pybullet.urdf")  


#################

INF = np.inf
PI = np.pi

# Params
ENG_RATE = 30 # Hz
T_STEP = 1./ENG_RATE
CAMERA_RATE = 1 # Hz
MOTION_RATE = 0.5 # Hz
LINE_LIFE = 5 # s
prev_pose = [0,0,0]
a = 0
inc = 6/(ENG_RATE/MOTION_RATE)
DEBUG = True
CAMERA_ACTIVATED = True
EEF_ID=8 # Naughty
SIM_T = 0.0
PRINT_SIM_T = False

CART_TOL = 0.005
ANGL_TOL = radians(1)

IKFastInfo = namedtuple('IKFastInfo', ['module_name', 'base_link', 'ee_link', 'free_joints'])
info = IKFastInfo(module_name='vs068_ikfast', base_link=0, ee_link=8, free_joints=[2, 3, 4, 5, 6, 7])

Pose = namedtuple('Pose', ['position', 'orientation'])

NEVER_COLLIDE_NAMES = [['world_to_base', 'base'],
                        ['base', 'joint0_1'],
                        ['joint0_1', 'joint1_2'],
                        ['joint1_2', 'joint2_3'],
                        ['joint2_3', 'joint3_4'],
                        ['joint3_4', 'joint4_5'],
                        ['joint4_5', 'joint5_eef'],
                        ['joint5_eef', 'base_finger_1'],
                        ['joint5_eef', 'base_finger_2'],
                        ['base_finger_1', 'base_finger_2']]

def set_IKFAST(onoff=True):
    global IKFAST_AVAILABLE
    IKFAST_AVAILABLE=onoff
    if IKFAST_AVAILABLE:
        try:
            import ikfast_vs068 as ikv
            IKFAST_AVAILABLE = True
        except ModuleNotFoundError:
            IKFAST_AVAILABLE = False
            
def get_IKFAST():
    global IKFAST_AVAILABLE
    return IKFAST_AVAILABLE