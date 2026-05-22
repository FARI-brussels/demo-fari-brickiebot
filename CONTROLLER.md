# Controller System Documentation

## Overview

This document explains how the Xbox controller interface works and how it communicates with the Arduino firmware to control the BrickieBot's stepper motors.

## System Architecture

```
Xbox Controller
      ↓ (USB/Bluetooth)
  pygame library
      ↓
xbox_controller.py (Python)
      ↓ (Serial @ 115200 baud)
controller.ino (Arduino)
      ↓ (GPIO pins)
Stepper Motor Drivers
      ↓
Physical Motors (X, Y, Z axes)
```

## Components

### 1. Xbox Controller Interface (`xbox_controller.py`)

The Python script uses the `pygame` library to read Xbox controller inputs and translate them into motor commands.

#### Controller Mapping

| Input | Axis/Button | Function |
|-------|-------------|----------|
| Left Stick X | Axis 0 | X-axis movement (left/right) |
| Left Stick Y | Axis 1 | Y-axis movement (forward/back) |
| Right Trigger | Axis 4 | Z-axis movement (up/down) |
| Start Button | Button 7 | Initiate homing sequence |
| B Button | Button 1 | Emergency stop |

#### Key Features

- **Deadzone**: 0.05 threshold to prevent drift
- **Speed Control**: Joystick position directly maps to motor speed (0.0 to 1.0)
- **Direction Control**: Joystick direction determines motor direction
- **Real-time Control**: 60 Hz update rate

### 2. Arduino Firmware (`controller.ino`)

The Arduino firmware controls three stepper motors using step/direction signals.

#### Hardware Configuration

- **Enable Pin**: Pin 8 (EN_PIN) - Enables/disables motor drivers
- **X-axis**: Step Pin 2, Direction Pin 5
- **Y-axis**: Step Pin 3, Direction Pin 6
- **Z-axis**: Step Pin 4, Direction Pin 7
- **Limit Switches**:
  - X-axis: Pin 37
  - Y-axis Left: Pin 10
  - Y-axis Right: Pin 11

#### Motion Parameters

```cpp
STEPS_PER_REV = 1600        // Steps per motor revolution
MM_PER_REV = 40             // Millimeters per revolution
MAX_X_POSITION_MM = 300     // Maximum X travel
MAX_Y_POSITION_MM = 400     // Maximum Y travel
BASE_STEP_DELAY = 500μs     // Slowest speed
MIN_STEP_DELAY = 100μs      // Fastest speed
```

## Communication Protocol

### Message Framing

All messages use a simple framing protocol to ensure reliable communication:

```
[START_BYTE] [MESSAGE_TYPE] [PAYLOAD...] [END_BYTE]
```

| Field | Value | Description |
|-------|-------|-------------|
| START_BYTE | 0xFF | Message start marker |
| END_BYTE | 0xFE | Message end marker |
| MSG_TYPE_POSITION | 0x01 | Position data from Arduino |
| MSG_TYPE_HOMING_COMPLETE | 0x02 | Homing finished notification |
| MSG_TYPE_COMMAND | 0x03 | Command from Python to Arduino |

### Command Structure (Python → Arduino)

```
[0xFF] [0x03] [COMMAND_BYTE] [X_SPEED] [Y_SPEED] [Z_SPEED] [0xFE]
     ↑      ↑         ↑           ↑          ↑          ↑        ↑
   Start  Type   Command    4 bytes    4 bytes    4 bytes    End
                  (1 byte)   (float)    (float)    (float)
```

#### Command Byte Encoding

The command byte is a bitfield that encodes step and direction for all three axes:

```
Bit 0: X_STEP (1 = step X motor)
Bit 1: X_DIR  (1 = positive direction, 0 = negative)
Bit 2: Y_STEP (1 = step Y motor)
Bit 3: Y_DIR  (1 = positive direction, 0 = negative)
Bit 4: Z_STEP (1 = step Z motor)
Bit 5: Z_DIR  (1 = positive direction, 0 = negative)
```

**Special Command**: `0x0F` = Initiate homing sequence

#### Speed Values

- Speeds are sent as 32-bit floating-point values (IEEE 754)
- Range: 0.0 (stopped) to 1.0 (maximum speed)
- The Arduino interpolates between `BASE_STEP_DELAY` and `MIN_STEP_DELAY`

### Position Updates (Arduino → Python)

```
[0xFF] [0x01] [X_POS] [Y_POS] [Z_POS] [0xFE]
     ↑      ↑      ↑       ↑       ↑      ↑
   Start  Type  4 bytes 4 bytes 4 bytes End
               (float) (float) (float)
```

- Positions are in millimeters
- Sent every 100 steps (`POSITION_UPDATE_INTERVAL`)
- Also forwarded to the simulation via UDP

### Homing Complete (Arduino → Python)

```
[0xFF] [0x02] [0xFE]
     ↑      ↑      ↑
   Start  Type   End
```

## Operation Flow

### 1. Initialization

1. Python script initializes pygame and detects Xbox controller
2. Opens serial connection to Arduino (115200 baud, `/dev/ttyACM0`)
3. Starts background thread for continuous movement commands
4. Automatically initiates homing sequence

### 2. Homing Sequence

When the home button is pressed or on startup:

1. Python sends `0x0F` command to Arduino
2. Arduino homes Y-axis first (moves until both limit switches trigger)
3. Then homes X-axis (moves until limit switch triggers)
4. Sets position to endstop offsets:
   - X: -325mm
   - Y: -150mm
   - Z: 0mm
5. Moves to center position (0, 0)
6. Sends homing complete message
7. Python script updates `is_homing` flag

### 3. Normal Operation

**Control Loop (60 Hz in main thread):**
```python
1. Read joystick axes and buttons
2. Apply deadzone filtering
3. Update speed and direction variables
4. Movement thread handles command sending
```

**Movement Thread (continuous):**
```python
1. Check for incoming position updates
2. If any speed > 0:
   a. Build command byte from directions
   b. Send batches of 32 commands with speed values
3. Repeat
```

**Arduino Loop (continuous):**
```python
1. Read incoming serial messages
2. Parse framed commands
3. Execute steps based on speed and timing
4. Send position updates every 100 steps
```

### 4. Safety Features

#### Python Side
- **Emergency Stop**: B button sets all speeds to 0
- **Deadzone**: Prevents unintended movement from stick drift
- **Thread Error Handling**: Catches and reports errors in movement thread

#### Arduino Side
- **Limit Switch Protection**: Prevents movement beyond endstops
- **Software Limits**: Enforces maximum travel distances
- **Homing Flag**: Blocks manual commands during homing

## Speed Control Algorithm

The Arduino converts speed values (0.0-1.0) to step delays:

```cpp
delay = BASE_STEP_DELAY - (speed × (BASE_STEP_DELAY - MIN_STEP_DELAY))
```

Example:
- Speed 0.0: delay = 500μs (slow)
- Speed 0.5: delay = 300μs (medium)
- Speed 1.0: delay = 100μs (fast)

Each axis independently calculates its delay, allowing for smooth simultaneous multi-axis movement.

## Integration with Simulation

The Python controller also sends position data to a MuJoCo simulation:

- **Protocol**: UDP
- **Port**: 12345
- **Format**: Three 32-bit floats (X, Y, Z) in network byte order
- **Purpose**: Real-time visualization of physical robot state

## Troubleshooting

### Controller Not Detected
```bash
# Check if controller is connected
ls /dev/input/js*
# Install dependencies
pip install pygame
```

### Serial Connection Issues
```bash
# Check Arduino connection
ls /dev/ttyACM*
# Add user to dialout group
sudo usermod -aG dialout $USER
```

### Motors Not Moving
1. Check that motors are enabled (EN_PIN is LOW)
2. Verify limit switches aren't triggered inappropriately
3. Ensure homing sequence completed successfully
4. Check serial monitor for error messages

## Development Notes

### Adding New Buttons/Axes

1. Find button/axis number using pygame test:
```python
print(f"Buttons: {joystick.get_numbuttons()}")
print(f"Axes: {joystick.get_numaxes()}")
```

2. Add mapping in `main()` loop
3. Update command byte if adding new motor axis

### Modifying Speed Characteristics

- Adjust `BASE_STEP_DELAY` and `MIN_STEP_DELAY` in Arduino code
- Tune deadzone threshold (currently 0.05) in Python code
- Modify command batch size (currently 32) for different responsiveness

### Communication Debugging

Enable verbose logging:
```python
# In xbox_controller.py, uncomment line 178:
print(self.x_speed, self.y_speed, self.z_speed)
```

Monitor Arduino serial output:
```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## Future Improvements

- [ ] Implement acceleration/deceleration curves for smoother motion
- [ ] Add support for multiple speed presets (fine/coarse control)
- [ ] Implement position-based control mode (not just velocity)
- [ ] Add haptic feedback for limit switch triggers
- [ ] Support for saving/replaying motion sequences





