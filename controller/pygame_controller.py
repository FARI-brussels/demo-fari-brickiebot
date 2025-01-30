import pygame
import serial
import time
import threading
from queue import Queue
import traceback
class StepperController:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        # Command definitions
        self.X_STEP = 1 << 0
        self.X_DIR = 1 << 1
        self.Y_STEP = 1 << 2
        self.Y_DIR = 1 << 3
        self.Z_STEP = 1 << 4
        self.Z_DIR = 1 << 5
        self.CMD_REQUEST_STATUS = 0xF0  # Match Arduino definition
        self.CMD_HOME = 0x0F           # Homing command (match with Arduino)
        self.STEP_DELAY = 0.0002  # Increased to reduce buffer overflow chances
        
        # Add flags for each axis movement
        self.moving_x = False
        self.moving_y = False
        self.moving_z = False
        self.x_direction = True
        self.y_direction = True
        self.z_direction = True
        self.is_homing = False  # New flag for homing status
        
        # Thread control
        self.running = True
        self.thread_error = None
        
        # Add counters for debugging
        self.movement_counter = 0
        self.last_debug_time = time.time()
        
        try:
            self.serial = serial.Serial(
                port, 
                baud_rate, 
                timeout=1,
                write_timeout=1
            )
            # Clear any pending data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(2)  # Wait for Arduino to reset
            print("Connected to Arduino")
            
            # Start the movement thread
            self.movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
            self.movement_thread.start()
            
        except serial.SerialException as e:
            raise Exception(f"Failed to connect to Arduino: {e}")

    def start_homing(self):
        """Initiate the homing sequence"""
        if not self.is_homing:
            print("Starting homing sequence...")
            try:
                self.is_homing = True
                # Stop any ongoing movements
                self.moving_x = False
                self.moving_y = False
                self.moving_z = False
                # Send homing command
                self.send_command(self.CMD_HOME)
            except Exception as e:
                print(f"Error starting homing sequence: {e}")
                self.is_homing = False
                raise

    def send_command(self, command):
        max_retries = 3
        retry_delay = 0.1
        
        for attempt in range(max_retries):
            try:
                if not self.serial.is_open:
                    raise serial.SerialException("Serial port is closed")
                
                if self.serial.out_waiting > 3600:
                    print(f"Buffer full ({self.serial.out_waiting} bytes), waiting...")
                    time.sleep(retry_delay)
                    continue
                    
                self.serial.write(bytes([command]))
                time.sleep(self.STEP_DELAY)
                self.movement_counter += 1
                return
                
            except serial.SerialTimeoutException:
                if attempt < max_retries - 1:
                    print(f"Write timeout, retry {attempt + 1}/{max_retries}")
                    self.serial.reset_output_buffer()
                    time.sleep(retry_delay)
                else:
                    print("Max retries reached, giving up")
                    raise
            except serial.SerialException as e:
                print(f"Serial error in send_command: {e}")
                self.thread_error = e
                raise

    def rotate_axis(self, step_bit, dir_bit, clockwise=True):
        try:
            command = dir_bit if clockwise else 0
            command = dir_bit | step_bit if clockwise else step_bit
            self.send_command(command)
            command = dir_bit if clockwise else 0
            self.send_command(command)
        except Exception as e:
            print(f"Error in rotate_axis: {e}")
            self.thread_error = e
            raise

    def _movement_loop(self):
        print("Movement thread started")
        debug_interval = 5  # Print debug info every 5 seconds
        last_buffer_clear = time.time()
        buffer_clear_interval = 0.1  # Clear buffer every 100ms
        
        while self.running:
            # Regularly clear input buffer and request status
            current_time = time.time()
            if current_time - last_buffer_clear >= buffer_clear_interval:
                self.serial.reset_input_buffer()  # Clear any old data
                try:
                    self.serial.write(bytes([self.CMD_REQUEST_STATUS]))
                    #time.sleep(0.01)
                    # Read response if available
                    while self.serial.in_waiting > 0:
                        print(self.serial.read())
                        if self.serial.read()[0] == 0xFF:
                            # Read until we get end marker
                            data = self.serial.read_until(bytes([0xFE])).decode('ascii')
                            if data:
                                parts = data[:-1].split(',')  # Remove end marker and split
                                if len(parts) == 4:
                                    limit_status = int(parts[0])
                                    self.x_position = int(parts[1])
                                    self.y_position = int(parts[2])
                                    self.z_position = int(parts[3])
                                    print(f"Positions - X: {self.x_position}, Y: {self.y_position}, Z: {self.z_position}")
                            break
                except Exception as e:
                    print(f"Error reading status: {e}")
                last_buffer_clear = current_time

            try:
                # Print debug information periodically
                current_time = time.time()
                if current_time - self.last_debug_time >= debug_interval:
                    print(f"Movement thread alive. Movements: {self.movement_counter}, "
                            f"Buffer usage: {self.serial.out_waiting} bytes")
                    self.last_debug_time = current_time
                    self.movement_counter = 0
                
                # Check each axis and perform movements
                if self.moving_x:
                    self.rotate_axis(self.X_STEP, self.X_DIR, self.x_direction)
                if self.moving_y:
                    self.rotate_axis(self.Y_STEP, self.Y_DIR, self.y_direction)
                if self.moving_z:
                    self.rotate_axis(self.Z_STEP, self.Z_DIR, self.z_direction)
                
                # Only sleep if we're not moving anything
                if not (self.moving_x or self.moving_y or self.moving_z):
                    time.sleep(self.STEP_DELAY)
                    
            except Exception as e:
                print(f"Error in movement loop: {e}")
                print(traceback.format_exc())
                self.thread_error = e
                break
        
        print("Movement thread ending")
    def close(self):
        print("Closing controller...")
        self.running = False
        time.sleep(0.1)  # Give thread time to finish
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.reset_output_buffer()  # Clear any pending data
            self.serial.close()
        print("Serial connection closed")

# Main function remains the same as in your original code

def main():
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Stepper Controller")
    clock = pygame.time.Clock()
    running = True
    
    try:
        controller = StepperController()
        last_thread_check = time.time()
        
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
                    if event.key == pygame.K_h:  # Add homing command
                        controller.start_homing()
                    elif not controller.is_homing:  # Only allow movement if not homing
                        if event.key == pygame.K_LEFT:
                            controller.moving_x = True
                            controller.x_direction = False
                        elif event.key == pygame.K_RIGHT:
                            controller.moving_x = True
                            controller.x_direction = True
                        elif event.key == pygame.K_UP:
                            controller.moving_y = True
                            controller.y_direction = True
                        elif event.key == pygame.K_DOWN:
                            controller.moving_y = True
                            controller.y_direction = False
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

            clock.tick(60)

    except Exception as e:
        print(f"Main loop error: {e}")
        print(traceback.format_exc())
    finally:
        controller.close()
        pygame.quit()

if __name__ == "__main__":
    main()