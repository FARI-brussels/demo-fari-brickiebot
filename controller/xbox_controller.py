import pygame
import serial
import time
import threading
import traceback
import socket
import struct

class StepperController:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        # Message framing constants
        self.MSG_START = 0xFF
        self.MSG_END = 0xFE
        self.MSG_TYPE_POSITION = 0x01
        self.MSG_TYPE_HOMING_COMPLETE = 0x02
        self.MSG_TYPE_COMMAND = 0x03
        self.CMD_SET_VELOCITY = 0x10
        
        # Command constants
        self.CMD_HOME = 0x80
        
        # Speed control
        self.x_speed = 0.0
        self.y_speed = 0.0
        self.z_speed = 0.0
        self.HOMING_SPEED = 0.5
        self.x_direction = False
        self.y_direction = False
        self.z_direction = True
        
        self.is_homing = False
        self.running = True
        self.thread_error = None
        self.position_lock = threading.Lock()
        
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.simulation_address = ('localhost', 12345)  # UDP port for simulation
        
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
        
    
    def read_message(self):
        """Read and parse a complete message from Arduino"""
        try:
            # Wait for start marker
            while True:
                if self.serial.in_waiting:
                    byte = self.serial.read()
                    if byte and byte[0] == self.MSG_START:
                        break
            
            # Read message type
            if self.serial.in_waiting:
                msg_type = self.serial.read()
                if not msg_type:
                    return None
                
                if msg_type[0] == self.MSG_TYPE_POSITION:
                    # Read position data (12 bytes total: 4 bytes for each axis)
                    data = self.serial.read(12)
                    if len(data) == 12:
                        # Wait for end marker
                        end_marker = self.serial.read()
                        if end_marker and end_marker[0] == self.MSG_END:
                            x = struct.unpack('f', data[0:4])[0]
                            y = struct.unpack('f', data[4:8])[0]
                            z = struct.unpack('f', data[8:12])[0]
                            
                            with self.position_lock:
                                self.x_position = x
                                self.y_position = y
                                self.z_position = z
                            
                            # Send position to simulation
                            position_data = struct.pack('!fff', x, y, z)
                            self.udp_socket.sendto(position_data, self.simulation_address)
                            return "position"
                
                elif msg_type[0] == self.MSG_TYPE_HOMING_COMPLETE:
                    # Read end marker
                    end_marker = self.serial.read()
                    if end_marker and end_marker[0] == self.MSG_END:
                        print("Homing sequence completed")
                        self.is_homing = False
                        return "homing_complete"
            
            return None
        
        except Exception as e:
            print(f"Error reading message: {e}")
            return None

    
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
                self.send_command(self.CMD_HOME, self.HOMING_SPEED, self.HOMING_SPEED, self.HOMING_SPEED)
            except Exception as e:
                print(f"Error starting homing sequence: {e}")
                self.is_homing = False
                raise
    
    def send_command(self, command, x_speed, y_speed, z_speed):
        """Send framed command with speed values to Arduino"""
        try:
            if not self.serial or not self.serial.is_open:
                raise serial.SerialException("Serial port is closed")
            
            # Pack speeds as floats
            speed_bytes = struct.pack('fff', x_speed, y_speed, z_speed)
            
            # Create framed message: START + TYPE + COMMAND + SPEEDS + END
            message = bytes([
                self.MSG_START,
                self.MSG_TYPE_COMMAND,
                command
            ]) + speed_bytes + bytes([self.MSG_END])
            
            self.serial.write(message)
            
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            self.thread_error = e
            self.running = False
    
    def _movement_loop(self):
        """Main movement loop: periodically send signed velocities"""
        print("Movement thread started")
        send_interval_s = 0.02  # 50 Hz
        last_send = time.time()
        while self.running:
            try:
                if self.serial.in_waiting >= 3:
                    self.read_message()

                now = time.time()
                if now - last_send >= send_interval_s:
                    if not self.is_homing:
                        self.send_command(
                            self.CMD_SET_VELOCITY,
                            float(self.x_speed if self.x_direction else -self.x_speed),
                            float(self.y_speed if self.y_direction else -self.y_speed),
                            float(self.z_speed if self.z_direction else -self.z_speed)
                        )
                    else:
                        # While homing, ensure commanded velocity is zero
                        self.send_command(self.CMD_SET_VELOCITY, 0.0, 0.0, 0.0)
                    last_send = now

                time.sleep(0.001)

            except Exception as e:
                print(f"Error in movement loop: {e}")
                print(traceback.format_exc())
                self.thread_error = e
                break

    
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
    controller.start_homing()
    
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
            try:
                x_axis = joystick.get_axis(0)
                y_axis = joystick.get_axis(1)
                z_axis = joystick.get_axis(4)
                home_button = joystick.get_button(7)
                emergency_stop = joystick.get_button(1)
            except pygame.error:
                print("Controller disconnected!")
                running = False
                continue
            
            if home_button:
                controller.start_homing()
            
            if emergency_stop:
                controller.x_speed = 0
                controller.y_speed = 0
                controller.z_speed = 0
                print("Emergency stop triggered!")
                continue
            print(x_axis, y_axis, z_axis)
            # Set speeds directly from joystick values
            controller.x_speed = abs(x_axis) if abs(x_axis) > 0.1 else 0
            controller.x_direction = x_axis < 0  # Inverted to match firmware direction convention
            
            controller.y_speed = abs(y_axis) if abs(y_axis) > 0.1 else 0
            controller.y_direction = y_axis < 0  
            controller.z_speed = abs(z_axis) if abs(z_axis) > 0.1 else 0
            controller.z_direction = z_axis > 0  
            
            clock.tick(60)
    
    except Exception as e:
        print(f"Main loop error: {e}")
        print(traceback.format_exc())
    
    finally:
        controller.close()
        pygame.quit()

if __name__ == "__main__":
    main()
