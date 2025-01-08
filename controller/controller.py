import serial
import time

class StepperController:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        # Motor configuration
        self.STEPS_PER_REVOLUTION = 8000  # Standard for many stepper motors, adjust as needed
        self.STEP_DELAY = 0.0005  # Delay between steps in seconds
        
        # Command bit masks
        self.X_STEP = 1 << 0
        self.X_DIR = 1 << 1
        self.Y_STEP = 1 << 2
        self.Y_DIR = 0 << 3
        self.Z_STEP = 1 << 4
        self.Z_DIR = 1 << 5
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print("Connected to Arduino")
        except serial.SerialException as e:
            raise Exception(f"Failed to connect to Arduino: {e}")

    def send_command(self, command):
        """Send a single byte command to Arduino"""
        self.serial.write(bytes([command]))
        time.sleep(self.STEP_DELAY)  # Small delay between commands

    def rotate_axis(self, step_bit, dir_bit, steps, clockwise=True):
        """Rotate a specific axis by given number of steps"""
        # Set direction
        command = dir_bit if clockwise else 0
        self.send_command(command)
        
        # Perform steps
        for _ in range(steps):
            # Step high
            command = dir_bit | step_bit if clockwise else step_bit
            self.send_command(command)
            
            # Step low (reset step bit)
            command = dir_bit if clockwise else 0
            self.send_command(command)

    def rotate_all_axes(self):
        """Perform one revolution on each axis sequentially"""
        try:
            """            # X axis revolution
            print("Rotating X axis...")
            self.rotate_axis(self.X_STEP, self.X_DIR, self.STEPS_PER_REVOLUTION)
            time.sleep(0.5)  # Pause between axes
            """
            """
            # Y axis revolution
            print("Rotating Y axis...")
            self.rotate_axis(self.Y_STEP, self.Y_DIR, self.STEPS_PER_REVOLUTION)
            time.sleep(0.5)

            """
            # Z axis revolution
            print("Rotating Z axis...")
            self.rotate_axis(self.Z_STEP, self.Z_DIR, self.STEPS_PER_REVOLUTION)
            
            
            print("All rotations completed!")
            
        except Exception as e:
            print(f"Error during rotation: {e}")
            
    def close(self):
        """Close the serial connection"""
        if self.serial.is_open:
            self.serial.close()
            print("Serial connection closed")

def main():
    controller = None
    try:
        # Create controller instance
        controller = StepperController()
        
        # Perform rotations
        controller.rotate_all_axes()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if controller:
            controller.close()

if __name__ == "__main__":
    main()