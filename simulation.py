# %%
import swift
from math import pi
import roboticstoolbox as rtb
from spatialgeometry import Mesh
from spatialmath import SE3, Twist3
from spatialmath.base import *
from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import copy
import os
import time

def jacobian_i_k_optimisation(robot, v, qd_max=1):
    # jacobian inverse kinematics with optimisation
    J = robot.jacobe(robot.q)
    prog = MathematicalProgram()
    qd_opt = prog.NewContinuousVariables(6, "v_opt")
    # Define the error term for the cost function
    error = J @ qd_opt - v
    prog.AddCost(error.dot(error))
    # Add bounding box constraint for joint velocities
    lower_bounds = [-qd_max] * 6  # Lower bounds for each joint velocity
    upper_bounds = [qd_max] * 6   # Upper bounds for each joint velocity
    prog.AddBoundingBoxConstraint(lower_bounds, upper_bounds, qd_opt)
    # Solve the optimization problem
    result = Solve(prog)
    return result.is_success(), result.GetSolution(qd_opt)



crane = Mesh(
    filename=str(f"{os.path.expanduser('~')}/Documents/FARI/repositories/demo-fari-brickiebot/crane_body.glb"),
    color=[34, 143, 201],
    scale=(0.001,) * 3,

)
end_effector = Mesh(
    filename=str(f"{os.path.expanduser('~')}/Documents/FARI/repositories/demo-fari-brickiebot/end_effector.glb"),
    color=[31, 184, 72],
    scale=(0.001,) * 3,
)

shaft = Mesh(
    filename=str(f"{os.path.expanduser('~')}/Documents/FARI/repositories/demo-fari-brickiebot/shaft.glb"),
    color=[31, 184, 72],
    scale=(0.001,) * 3,
)

rails= Mesh(
    filename=str(f"{os.path.expanduser('~')}/Documents/FARI/repositories/demo-fari-brickiebot/rails.glb"),
    color=[240, 103, 103],
    scale=(0.001,) * 3,
)

brick = Mesh(
    filename=str(f"{os.path.expanduser('~')}/Documents/FARI/repositories/demo-fari-brickiebot/brick.glb"),
    color=[50, 50, 50],
    scale=(0.001,) * 3,
)

brickwall = []
for i in range(4):
    for j in range(3):
        if not (i==3 and j==0):
            b = copy.copy(brick)
            b.T = SE3(0, 0.2 + 0.06*j, 0.03*i)
            brickwall.append(b)


brick.T = SE3(0.2, 0.3, 0)

shaft_radius = 0.02
lite6 = rtb.models.URDF.Lite6()
lite6.base = SE3(0.4, 0, 0.0)*SE3.Rz(pi/2)

elephant = rtb.models.URDF.Mycobot280()
elephant.base = SE3(0.12, 0.05, 0.0)*SE3.Rz(pi)
elephant.q = [0., 0, -pi/2, 0, 0, 0]
env = swift.Swift()
env.launch(realtime=True)
time.sleep(5)

env.add(crane)
for b in brickwall:
    env.add(b)
env.add(brick)
env.add(end_effector)
env.add(rails)
env.add(lite6)
env.add(shaft)
#env.add(elephant)



# %%
end_effector.T = SE3(0, 0, 0.0)
crane.T = SE3(0, 0, 0.0)
brick.T = SE3(0.2, 0.3, 0)
f=30

def robot_move_to(robot, simulation, dt, dest, gain=2, treshold=0.001, qd_max=1, move_brick=False): 
        arrived = False
        while not arrived:

            q = robot.q
            if move_brick:
                brick.T = SE3(robot.fkine(q).t)
            if isinstance(dest, SE3) or (isinstance(dest, np.ndarray) and dest.shape==(4,4)):
                v, arrived = rtb.cp_servo(robot.fkine(q), dest, gain=gain, threshold=treshold)
                qd = jacobian_i_k_optimisation(robot, v, qd_max=qd_max)[1]
            else:
                qd, arrived = rtb.jp_servo(q, dest, gain=gain, threshold=treshold)
            robot.qd = qd
            simulation.step(dt)

        return arrived, robot.q


def crane_move_to(T_dest, n_sample, move_brick=False):
    traj = rtb.ctraj(SE3(end_effector.T), T_dest, n_sample)
    
    for i in range(100):
        
        crane.T = SE3.Tx(traj[i].x)
        end_effector.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)
        shaft.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)
        twist = Twist3.UnitRevolute([1 ,0, 0],[0, traj[i].y, 0.3785], 0)
        shaft.T = twist.SE3(traj[i].z/shaft_radius)*shaft.T
        if move_brick:
            brick.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)*SE3.Tz(traj[i].z)
        env.step(1/f)

T_pick = SE3(brick.T)
T_place_up = SE3(0.0, 0.2, 0.2)
T_place = SE3(0, 0.2, 0.09)
def crane_pick_and_place(T_pick, T_place_up, T_place, n_sample):
    crane_move_to(T_pick, n_sample)
    crane_move_to(T_place_up, n_sample, move_brick=True)
    robot_move_to(lite6, env, 1/f, T_place_up*SE3.RPY([0, 0, -90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1)
    robot_move_to(lite6, env, 1/f, T_place*SE3.RPY([0, 0, -90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1, move_brick=True)
    robot_move_to(lite6, env, 1/f, lite6.qz, gain=2, treshold=0.001, qd_max=1)
    #robot_move_to(elephant, env, 1/f, T_place_up*SE3.RPY([90, 180, 90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1)
    #robot_move_to(elephant, env, 1/f, T_place*SE3.RPY([90, 180, 90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1, move_brick=True)
    #robot_move_to(elephant, env, 1/f, [0., 0, -pi/2, 0, 0, 0], gain=2, treshold=0.001, qd_max=1)
    
crane_pick_and_place(T_pick, T_place_up, T_place, 100)


# %%







