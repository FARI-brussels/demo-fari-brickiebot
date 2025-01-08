import serial
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
STEPS_PER_REV = 2000    # Adjust based on your stepper motor and microstepping settings
STEP_DELAY = 0.001      # 10ms delay between steps, adjust as needed

# Helper function to construct the command byte
def make_cmd(x_step=0, x_dir=1, y_step=0, y_dir=1, z_step=0, z_dir=1):
    cmd = (x_step & 1) \
          | ((x_dir & 1) << 1) \
          | ((y_step & 1) << 2) \
          | ((y_dir & 1) << 3) \
          | ((z_step & 1) << 4) \
          | ((z_dir & 1) << 5)
    return cmd

# Helper functions to move a single axis
def move_axis(ser, axis='X', steps=200, direction=1):
    """
    Move the specified axis a number of steps in the given direction.
    axis: 'X', 'Y', or 'Z'
    direction: 1 = forward, 0 = backward
    """
    if axis == 'X':
        for _ in range(steps):
            # Send a command with x_step=1, x_dir=direction
            cmd = make_cmd(x_step=1, x_dir=direction, y_step=0, y_dir=1, z_step=0, z_dir=1)
            ser.write(bytes([cmd]))
            time.sleep(STEP_DELAY)
    elif axis == 'Y':
        for _ in range(steps):
            # Send a command with y_step=1, y_dir=direction
            cmd = make_cmd(x_step=0, x_dir=1, y_step=1, y_dir=direction, z_step=0, z_dir=1)
            ser.write(bytes([cmd]))
            time.sleep(STEP_DELAY)
    elif axis == 'Z':
        for _ in range(steps):
            # Send a command with z_step=1, z_dir=direction
            cmd = make_cmd(x_step=0, x_dir=1, y_step=0, y_dir=1, z_step=1, z_dir=direction)
            ser.write(bytes([cmd]))
            time.sleep(STEP_DELAY)

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait a moment for the Arduino to reset and be ready

    print("Starting test motion...")

    # X axis: one revolution forward
    print("X axis: one revolution forward")
    move_axis(ser, axis='X', steps=STEPS_PER_REV, direction=1)

    # X axis: one revolution backward
    print("X axis: one revolution backward")
    move_axis(ser, axis='X', steps=STEPS_PER_REV, direction=0)

    # Y axis: one revolution forward
    print("Y axis: one revolution forward")
    move_axis(ser, axis='Y', steps=STEPS_PER_REV, direction=1)

    # Y axis: one revolution backward
    print("Y axis: one revolution backward")
    move_axis(ser, axis='Y', steps=STEPS_PER_REV, direction=0)

    # Z axis: one revolution forward
    print("Z axis: one revolution forward")
    move_axis(ser, axis='Z', steps=STEPS_PER_REV, direction=1)

    # Z axis: one revolution backward
    print("Z axis: one revolution backward")
    move_axis(ser, axis='Z', steps=STEPS_PER_REV, direction=0)

    print("Test motion complete.")
    ser.close()

if __name__ == "__main__":
    main()
