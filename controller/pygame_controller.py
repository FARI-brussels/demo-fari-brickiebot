import pygame
import serial
import time
import threading
from queue import Queue
import traceback
import socket
import struct


class StepperController:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        # Command definitions
        self.X_STEP = 1 << 0
        self.X_DIR = 1 << 1
        self.Y_STEP = 1 << 2
        self.Y_DIR = 1 << 3
        self.Z_STEP = 1 << 4
        self.Z_DIR = 1 << 5
        self.CMD_HOME = 0x0F
        
        # Position tracking
        self.x_position = 0
        self.y_position = 0
        self.z_position = 0
        
        # Movement flags
        self.moving_x = False
        self.moving_y = False
        self.moving_z = False
        self.x_direction = True
        self.y_direction = True
        self.z_direction = True
        self.is_homing = False
        
        # Thread control
        self.running = True
        self.thread_error = None
        self.position_lock = threading.Lock()
        
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.simulation_address = ('localhost', 12345)  # UDP port for simulation

        
        try:
            self.serial = serial.Serial(
                port, 
                baud_rate, 
                timeout=1,
                write_timeout=1
            )
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(2)  # Wait for Arduino to reset
            print("Connected to Arduino")
            
            self.movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
            self.movement_thread.start()
            
        except serial.SerialException as e:
            raise Exception(f"Failed to connect to Arduino: {e}")

    def read_position_data(self):
        """Read and parse binary position data from Arduino"""
        try:
            # Wait for start marker
            while True:
                if self.serial.in_waiting:
                    if self.serial.read() == b'\xff':
                        break
            
            # Read position data (12 bytes total: 4 bytes for each axis)
            data = self.serial.read(12)
            if len(data) == 12:
                import struct
                # Use 'f' format for 32-bit float instead of 'i' for integer
                x = struct.unpack('f', data[0:4])[0]
                y = struct.unpack('f', data[4:8])[0]
                z = struct.unpack('f', data[8:12])[0]
                
                # Read end marker
                end_marker = self.serial.read()
                if end_marker == b'\xfe':
                    with self.position_lock:
                        self.x_position = x
                        self.y_position = y
                        self.z_position = z
                        
                    # Pack and send position data via UDP
                    position_data = struct.pack('!fff', x, y, z)  # Network byte order
                    self.udp_socket.sendto(position_data, self.simulation_address)
                    return True
                return False
        except Exception as e:
            print(f"Error reading/sending position: {e}")
            return False

    def send_command(self, command):
        """Send command to Arduino"""
        try:
            if not self.serial.is_open:
                raise serial.SerialException("Serial port is closed")
            
            self.serial.write(bytes([command]))
            return True
                
        except serial.SerialException as e:
            print(f"Serial error in send_command: {e}")
            self.thread_error = e
            raise

    def start_homing(self):
        """Initiate the homing sequence"""
        if not self.is_homing:
            print("Starting homing sequence...")
            try:
                self.is_homing = True
                self.moving_x = False
                self.moving_y = False
                self.moving_z = False
                self.send_command(self.CMD_HOME)
                self.read_position_data()  # Wait for final position update
                self.is_homing = False
            except Exception as e:
                print(f"Error during homing: {e}")
                self.is_homing = False
                raise

    def _movement_loop(self):
        """Main movement loop with rapid command sending"""
        print("Movement thread started")
        command_batch_size = 32  # Send multiple commands per iteration
        
        while self.running:
            try:
                if self.moving_x or self.moving_y or self.moving_z:
                    # Send multiple commands at once for faster movement
                    for _ in range(command_batch_size):
                        command = 0
                        if self.moving_x:
                            command |= self.X_STEP
                            if self.x_direction:
                                command |= self.X_DIR
                        if self.moving_y:
                            command |= self.Y_STEP
                            if self.y_direction:
                                command |= self.Y_DIR
                        if self.moving_z:
                            command |= self.Z_STEP
                            if self.z_direction:
                                command |= self.Z_DIR
                        
                        if command != 0:
                            self.send_command(command)
                
                # Check for position updates
                if self.serial.in_waiting > 13:  # Start marker + 12 bytes + end marker
                    self.read_position_data()
                else:
                    time.sleep(0.001)  # Short sleep if no movement or data
                    
            except Exception as e:
                print(f"Error in movement loop: {e}")
                print(traceback.format_exc())
                self.thread_error = e
                break
        
        print("Movement thread ending")

    def get_position(self):
        """Get current position of all axes"""
        with self.position_lock:
            return (self.x_position, self.y_position, self.z_position)

    def close(self):
        print("Closing controller...")
        self.running = False
        time.sleep(0.1)
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.reset_output_buffer()
            self.serial.close()
        if hasattr(self, 'udp_socket'):
            self.udp_socket.close()
        print("Connections closed")

    

def main():
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Stepper Controller")
    font = pygame.font.Font(None, 36)
    clock = pygame.time.Clock()
    running = True
    
    try:
        controller = StepperController()
        last_thread_check = time.time()
        controller.start_homing()
        while running:
            current_time = time.time()
            if current_time - last_thread_check >= 1.0:
                if not controller.movement_thread.is_alive():
                    print("Movement thread died!")
                    if controller.thread_error:
                        print(f"Thread error: {controller.thread_error}")
                    break
                last_thread_check = current_time

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_h:
                        controller.start_homing()
                    elif not controller.is_homing:
                        if event.key == pygame.K_LEFT:
                            controller.moving_x = True
                            controller.x_direction = False
                        elif event.key == pygame.K_RIGHT:
                            controller.moving_x = True
                            controller.x_direction = True
                        elif event.key == pygame.K_UP:
                            controller.moving_y = True
                            controller.y_direction = False
                        elif event.key == pygame.K_DOWN:
                            controller.moving_y = True
                            controller.y_direction = True
                        elif event.key == pygame.K_a:
                            controller.moving_z = True
                            controller.z_direction = True
                        elif event.key == pygame.K_z:
                            controller.moving_z = True
                            controller.z_direction = False
                
                elif event.type == pygame.KEYUP:
                    if event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                        controller.moving_x = False
                    elif event.key in (pygame.K_UP, pygame.K_DOWN):
                        controller.moving_y = False
                    elif event.key in (pygame.K_a, pygame.K_z):
                        controller.moving_z = False

            # Display position
            screen.fill((0, 0, 0))
            x, y, z = controller.get_position()
            position_text = font.render(f"X: {x} Y: {y} Z: {z}", True, (255, 255, 255))
            screen.blit(position_text, (10, 10))
            pygame.display.flip()

            clock.tick(60)

    except Exception as e:
        print(f"Main loop error: {e}")
        print(traceback.format_exc())
    finally:
        controller.close()
        pygame.quit()

if __name__ == "__main__":
    main()