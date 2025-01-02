import mujoco
import mujoco.viewer
import time
import pygame
import roboticstoolbox as rtb
from spatialmath import SE3, Twist3, UnitQuaternion, SO3
from spatialmath.base import *
from pydrake.solvers import MathematicalProgram, Solve


# Create MuJoCo model and data
model = mujoco.MjModel.from_xml_path("briekiebot.xml")
data = mujoco.MjData(model)

lite6_mujoco = model.body('link_base')
lite6_mujoco.pos = [-0.3, 0, 0]
lite6_rtb =  rtb.models.URDF.Lite6()
#lite6_rtb.base = lite6_rtb.base*SE3.Ty(0)
moving_robot = False

def jacobian_i_k_optimisation(robot, v, qd_max=1):
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

with mujoco.viewer.launch_passive(model, data) as viewer:
    #Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode((100, 100))
    start = time.time()
    
    # Position and rotation of the main camera
    viewer.cam.trackbodyid = 15
    viewer.cam.distance = 1.3
    viewer.cam.lookat = [0, 0, 0]
    viewer.cam.elevation = -55
    #viewer.cam.azimuth = 45
    
    simulation_action='init'
    while viewer.is_running():
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        keys = pygame.key.get_pressed()
        v = [0.0, 0.0, 0.0]

        if keys[pygame.K_UP]:
            v[2] = 0.0001
        if keys[pygame.K_DOWN]:
            v[2] = -0.0001
        if keys[pygame.K_LEFT]:
            v[0] = 0.0001
        if keys[pygame.K_RIGHT]:
            v[0] = -0.0001
        if keys[pygame.K_a]:
            v[1] = 0.0001
        if keys[pygame.K_z]:
            v[1] = -0.0001
        if keys[pygame.K_p] and not moving_robot:
            moving_robot = True
            arrived=False
            lite6_rtb.q = data.qpos[:6]   
            dest = lite6_rtb.fkine(lite6_rtb.q)*SE3.Ty(0.4)
            dest = SE3.RPY([0, 0, 180],order='xyz', unit='deg')*SE3.Ty(0)*SE3.Tz(0.44)*SE3.Tx(0.08699)*lite6_rtb.base
            n=0
            while not arrived: 
                n+=1
                if n%100==0:
                    v, arrived = rtb.cp_servo(lite6_rtb.fkine(lite6_rtb.q), dest, gain=1, threshold=0.1)
                    ik = jacobian_i_k_optimisation(lite6_rtb, v, qd_max=6)
                    data.ctrl[:6] = ik[1]
                    
                    lite6_rtb.q = data.qpos[:6]
                mujoco.mj_step(model, data)
                viewer.sync()
            data.ctrl[:6] = [0]*6
            print(arrived)
        
        #data.ctrl=[v[0], v[1]]
        """
        model.body('crane_body').pos += [v[1],0,0]
        model.body('end_effector').pos += [0,v[0],0]
        """
        
        mujoco.mj_step(model, data)
        viewer.sync()
        #time.sleep(model.opt.timestep)
            
    
def follow():
    pass