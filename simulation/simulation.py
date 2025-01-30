import mujoco
import mujoco.viewer
import multiprocessing
import numpy as np
import time
import sys


import numpy as np
from scipy.spatial.transform import Rotation as R

def rotate_y(quat, angle_deg):
    # Convert angle to radians
    angle_rad = np.radians(angle_deg) / 2

    # Incremental quaternion for Y-axis rotation
    q_increment = np.array([
        np.cos(angle_rad),  # w
        0,                  # x
        np.sin(angle_rad),  # y
        0                   # z
    ])

    # Multiply quaternions: q_new = q_increment * q_current
    w1, x1, y1, z1 = q_increment
    w2, x2, y2, z2 = quat

    q_new = np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,  # w
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,  # x
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,  # y
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2   # z
    ])

    return q_new


class MujocoSimulation:
    def __init__(self, command_queue, status_queue):
        self.command_queue = command_queue
        self.status_queue = status_queue
        self.running = True
        
    def setup_simulation(self):
        """Initialize MuJoCo simulation"""
        print("Setting up MuJoCo simulation...")  # This will be captured
        self.model = mujoco.MjModel.from_xml_path("/home/fari/Documents/demo-fari-brickiebot/simulation/briekiebot.xml")
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data, show_right_ui=True, show_left_ui=False)
        print(dir(self.viewer))
        print("Simulation initialized successfully")  # This will be captured
        self.lite6_mujoco = self.model.body('link_base')
        self.lite6_mujoco.pos = [-0.3, 0, 0]
        #lite6_rtb =  rtb.models.URDF.Lite6()
        # Set up camera and initial positions
        self.viewer.cam.trackbodyid = 15
        self.viewer.cam.distance = 2
        self.viewer.cam.lookat = [0, 0, 0]
        self.viewer.cam.elevation = -35
        self.viewer.cam.azimuth = -55
    
    def process_command(self, cmd_type, cmd_data):
        """Process incoming commands"""
        print(f"Processing command: type={cmd_type}, data={cmd_data}")  # This will be captured
        if cmd_type == 0x01:  # Movement command
            x_step = (cmd_data >> 0) & 1
            x_dir = (cmd_data >> 1) & 1
            y_step = (cmd_data >> 2) & 1
            y_dir = (cmd_data >> 3) & 1
            z_step = (cmd_data >> 4) & 1
            z_dir = (cmd_data >> 5) & 1
            
            step_size = 0.002
            v = [0.0, 0.0, 0.0]
            if x_step:
                v[0] = step_size if x_dir else -step_size
            if y_step:
                v[1] = step_size if y_dir else -step_size
            if z_step:
                v[2] = 2 if z_dir else -2
            
            #print(f"Applying velocities: {v}")  # This will be captured
            self.data.ctrl[-3:] = [v[0], v[1], v[2]]
            self.model.body('crane_body').pos += [0, v[1], 0]
            self.model.body('end_effector').pos += [v[0], 0, 0]
            self.model.body("shaft").quat = rotate_y(self.model.body('shaft').quat, angle_deg=v[2])

    
    def run(self):
        """Main simulation loop"""
        try:
            self.setup_simulation()
            print("Starting simulation loop")  # This will be captured
            
            while self.running and self.viewer.is_running():
                # Process any pending commands
                while not self.command_queue.empty():
                    try:
                        cmd = self.command_queue.get_nowait()
                        self.process_command(cmd['type'], cmd['data'])
                    except Exception as e:
                        print(f"Error processing command: {e}")  # This will be captured
                
                # Update simulation
                mujoco.mj_step(self.model, self.data)
                self.viewer.sync()
                
                # Send status update
                try:
                    self.status_queue.put_nowait(self.get_status())
                except:
                    pass
                
                time.sleep(0.001)
        except Exception as e:
            print(f"Simulation error: {e}")  # This will be captured
        finally:
            print("Simulation ending")  # This will be captured
            if hasattr(self, 'viewer'):
                self.viewer.close()




def run_simulation(command_queue, status_queue):
    try:
        # Try to reconfigure stdout, but don't fail if it's not possible
        try:
            sys.stdout.reconfigure(line_buffering=True)
        except (AttributeError, IOError):
            pass  # Ignore if reconfigure is not available
            
        print("Simulation process started")
        sim = MujocoSimulation(command_queue, status_queue)
        sim.run()
        
    except Exception as e:
        print(f"Simulation error: {e}")