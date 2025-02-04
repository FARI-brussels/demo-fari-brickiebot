import mujoco
import mujoco.viewer
import multiprocessing
import numpy as np
import time
import sys
import socket
import struct


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
    def __init__(self):
        self.running = True
        # Add UDP socket setup
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', 12345))  # Listen on all interfaces
        self.udp_socket.setblocking(False)  # Non-blocking mode
        
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
        self.viewer.cam.lookat = [0, 0, 0.2]
        self.viewer.cam.elevation = -30
        self.viewer.cam.azimuth = 90

    def update_position_from_udp(self):
        """Check for and process any incoming UDP position updates"""
        try:
            data, addr = self.udp_socket.recvfrom(12)  # 12 bytes for 3 floats
            if len(data) == 12:
                x, y, z = struct.unpack('!fff', data)
                print(x,y,z)
                # Update MuJoCo model positions
                self.model.body('crane_body').pos[1] = y/1000
                self.model.body('end_effector').pos[0] = -x/1000
                # For Z, we might need to convert to rotation depending on your setup
                angle = z * (180.0 / np.pi)  # Convert to degrees if needed
                self.model.body("shaft").quat = rotate_y(
                    self.model.body('shaft').quat, 
                    angle_deg=angle
                )
        except BlockingIOError:
            pass  # No data available
        except Exception as e:
            print(f"Error receiving UDP data: {e}")
    
    
    def run(self):
        """Main simulation loop"""
        try:
            self.setup_simulation()
            print("Starting simulation loop")
            
            while self.running and self.viewer.is_running():
                # Check for UDP position updates
                self.update_position_from_udp()
                
                # Update simulation
                mujoco.mj_step(self.model, self.data)
                self.viewer.sync()
                
                # Send status update
                try:
                    self.status_queue.put_nowait(self.get_status())
                except:
                    pass
                
  
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            print("Simulation ending")
            if hasattr(self, 'viewer'):
                self.viewer.close()
            if hasattr(self, 'udp_socket'):
                self.udp_socket.close()




def run_simulation():
    try:
        # Try to reconfigure stdout, but don't fail if it's not possible
        try:
            sys.stdout.reconfigure(line_buffering=True)
        except (AttributeError, IOError):
            pass  # Ignore if reconfigure is not available
            
        print("Simulation process started")
        sim = MujocoSimulation()
        sim.run()
        
    except Exception as e:
        print(f"Simulation error: {e}")


run_simulation()