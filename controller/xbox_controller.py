import pygame
import serial
import time
import threading
import traceback

class StepperController:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        self.X_STEP = 1 << 0
        self.X_DIR = 1 << 1
        self.Y_STEP = 1 << 2
        self.Y_DIR = 1 << 3
        self.Z_STEP = 1 << 4
        self.Z_DIR = 1 << 5
        self.CMD_REQUEST_STATUS = 0xF0
        self.CMD_HOME = 0x0F
        self.STEP_DELAY = 0.001  # Increased for stability
        
        self.moving_x = False
        self.moving_y = False
        self.moving_z = False
        self.x_direction = True
        self.y_direction = True
        self.z_direction = True
        self.is_homing = False
        
        self.running = True
        self.thread_error = None
        
        try:
            self.serial = serial.Serial(port, baud_rate, timeout=1, write_timeout=1)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(2)
            print("Connected to Arduino")
            
            self.movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
            self.movement_thread.start()
        except serial.SerialException as e:
            raise Exception(f"Failed to connect to Arduino: {e}")
    
    def send_command(self, command):
        try:
            if not self.serial or not self.serial.is_open:
                raise serial.SerialException("Serial port is closed")
            self.serial.write(bytes([command]))
            time.sleep(self.STEP_DELAY)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.thread_error = e
            self.running = False
    
    def rotate_axis(self, step_bit, dir_bit, clockwise=True):
        try:
          command = dir_bit | step_bit if clockwise else step_bit
          self.send_command(command)
          self.send_command(0)
          # self.send_command(step_bit)
        except Exception as e:
            print(f"Error in rotate_axis: {e}")
            self.thread_error = e
            raise
    
    def _movement_loop(self):
        while self.running:
            try:
                if self.moving_x:
                    self.rotate_axis(self.X_STEP, self.X_DIR, self.x_direction)
                if self.moving_y:
                    self.rotate_axis(self.Y_STEP, self.Y_DIR, self.y_direction)
                if self.moving_z:
                    self.rotate_axis(self.Z_STEP, self.Z_DIR, self.z_direction)
                time.sleep(self.STEP_DELAY)
            except Exception as e:
                print(f"Error in movement loop: {e}")
                print(traceback.format_exc())
                self.thread_error = e
                break
    
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
    
    def close(self):
        print("Closing controller...")
        self.running = False
        time.sleep(0.1)
        if self.serial.is_open:
            self.serial.reset_output_buffer()
            self.serial.close()
        print("Serial connection closed")


def main():
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No Xbox controller detected!")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Using controller: {joystick.get_name()}")
    
    controller = StepperController()
    clock = pygame.time.Clock()
    running = True
    
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            try:
                x_axis = joystick.get_axis(0)
                y_axis = joystick.get_axis(1)
                z_axis = joystick.get_axis(3)
                home_button = joystick.get_button(7)
                emergency_stop = joystick.get_button(1)  # B button for stop
            except pygame.error:
                print("Controller disconnected!")
                running = False
                continue
            
            if home_button:
                controller.start_homing()
            
            if emergency_stop:
                controller.moving_x = False
                controller.moving_y = False
                controller.moving_z = False
                print("Emergency stop triggered!")
                continue
            
            controller.moving_x = abs(x_axis) > 0.2
            controller.x_direction = x_axis > 0
            
            controller.moving_y = abs(y_axis) > 0.2
            controller.y_direction = y_axis < 0  # Invert for natural up/down
            
            controller.moving_z = abs(z_axis) > 0.2
            controller.z_direction = z_axis < 0  # Invert for natural up/down
            
            clock.tick(60)
    
    except Exception as e:
        print(f"Main loop error: {e}")
        print(traceback.format_exc())
    
    finally:
        controller.close()
        pygame.quit()

if __name__ == "__main__":
    main()
