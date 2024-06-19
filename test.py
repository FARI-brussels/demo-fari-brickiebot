
#%%
import swift
from math import pi
import roboticstoolbox as rtb
from spatialgeometry import Mesh, Axes
from spatialmath import SE3, UnitQuaternion
from spatialmath.base import *
import numpy as np
import copy
import os
import time

l = rtb.models.URDF.Lite6()

env = swift.Swift()
env.launch(realtime=True)
rails = Mesh(
    filename=str(f"{os.path.expanduser('~')}/Documents/FARI/repositories/demo-fari-brickiebot/rails.glb"),
    color=[34, 143, 201],
)
time.sleep(5)
#rails.T = rails.T*SE3.Rx(pi/2)
#env.add(l)
env.add(rails)

# %%
