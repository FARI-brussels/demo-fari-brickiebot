#define EN_PIN 8
#define STEP_PIN_X 2
#define DIR_PIN_X 5
#define STEP_PIN_Y 3
#define DIR_PIN_Y 6
#define STEP_PIN_Z 4
#define DIR_PIN_Z 7
#define LIMIT_SWITCH_X 37
#define LIMIT_SWITCH_Y_L 10
#define LIMIT_SWITCH_Y_R 11
#define LIMIT_SWITCH_Z 9

#define HOME_DIR_X HIGH   
#define HOME_DIR_Y HIGH
#define HOME_DIR_Z LOW    
#define HOMING_SPEED 500  
#define HOME_COMMAND 0x80  // Changed from 0x0F to avoid accidental triggering 

#define MSG_START 0xFF
#define MSG_END 0xFE
#define MSG_TYPE_POSITION 0x01
#define MSG_TYPE_HOMING_COMPLETE 0x02
#define MSG_TYPE_COMMAND 0x03
#define CMD_SET_VELOCITY 0x10

#define BASE_STEP_DELAY 500
#define MIN_STEP_DELAY 250  // Maximum speed
#define POSITION_UPDATE_INTERVAL 100
#define BUFFER_SIZE 128

// Motion parameters
#define STEPS_PER_REV 1600
#define MM_PER_REV 40
#define MAX_X_POSITION_MM 300
#define MAX_Y_POSITION_MM 400
#define MAX_Z_POSITION_MM 300
#define X_ENDSTOP_POS -325
#define Y_ENDSTOP_POS -150
#define Z_ENDSTOP_POS -100

// Speed control: signed velocities in [−1.0, 1.0]
float x_speed = 0;
float y_speed = 0;
float z_speed = 0;
unsigned long last_x_step = 0;
unsigned long last_y_step = 0;
unsigned long last_z_step = 0;

bool is_homing = false;
long x_steps = 0;
long y_steps = 0;
long z_steps = 0;

// Buffer for receiving commands (type + cmd + 3 floats + end)
uint8_t receiveBuffer[14];
int receiveIndex = 0;
bool messageStarted = false;

// Step buffering
uint8_t commandBuffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferTail = 0;
int stepsUntilUpdate = POSITION_UPDATE_INTERVAL;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(DIR_PIN_Y, OUTPUT);
  pinMode(STEP_PIN_Z, OUTPUT);
  pinMode(DIR_PIN_Z, OUTPUT);
  
  pinMode(LIMIT_SWITCH_X, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Y_L, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Y_R, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Z, INPUT_PULLUP);
  
  digitalWrite(EN_PIN, LOW);
}

float stepsToMM(long steps) {
  return ((float)steps / STEPS_PER_REV) * MM_PER_REV;
}

long mmToSteps(float mm) {
  return (long)((mm * STEPS_PER_REV) / MM_PER_REV);
}

bool isXEndstopHit() {
  return digitalRead(LIMIT_SWITCH_X) == HIGH;
}

bool isYEndstopsHit() {
  return digitalRead(LIMIT_SWITCH_Y_L) == HIGH && digitalRead(LIMIT_SWITCH_Y_R) == HIGH;
}

bool isZEndstopHit() {
  return digitalRead(LIMIT_SWITCH_Z) == HIGH;
}

// canMove functions use DIR pin value directly
// Block movement if:
// - Going towards endstop (homing direction) and endstop is hit
// - Going away from endstop and max position is exceeded
bool canMoveX(uint8_t dir_pin) {
  if (dir_pin == HOME_DIR_X && isXEndstopHit()) {
    return false;
  }
  if (dir_pin != HOME_DIR_X && stepsToMM(x_steps) >= MAX_X_POSITION_MM) {
    return false;
  }
  return true;
}

bool canMoveY(uint8_t dir_pin) {
  if (dir_pin == HOME_DIR_Y && isYEndstopsHit()) {
    return false;
  }
  if (dir_pin != HOME_DIR_Y && stepsToMM(y_steps) >= MAX_Y_POSITION_MM) {
    return false;
  }
  return true;
}

bool canMoveZ(uint8_t dir_pin) {
  if (dir_pin == HOME_DIR_Z && isZEndstopHit()) {
    return false;
  }
  if (dir_pin != HOME_DIR_Z && stepsToMM(z_steps) >= MAX_Z_POSITION_MM) {
    return false;
  }
  return true;
}

void sendPosition() {
  union {
    float f;
    uint8_t b[4];
  } x_conv, y_conv, z_conv;
  
  x_conv.f = stepsToMM(x_steps);
  y_conv.f = stepsToMM(y_steps);
  z_conv.f = stepsToMM(z_steps);
  
  Serial.write(MSG_START);
  Serial.write(MSG_TYPE_POSITION);  // Indicate this is position data
  Serial.write(x_conv.b, 4);
  Serial.write(y_conv.b, 4);
  Serial.write(z_conv.b, 4);
  Serial.write(MSG_END);
}

bool bufferIsFull() {
  return ((bufferHead + 1) % BUFFER_SIZE) == bufferTail;
}

bool bufferIsEmpty() {
  return bufferHead == bufferTail;
}

void addToBuffer(uint8_t command) {
  if (!bufferIsFull()) {
    commandBuffer[bufferHead] = command;
    bufferHead = (bufferHead + 1) % BUFFER_SIZE;
  }
}

void stepFromVelocities() {
  if (is_homing) {
    return;
  }

  // Determine DIR pin values from speed sign
  // Positive speed = move away from endstop (opposite of HOME_DIR)
  // Negative speed = move towards endstop (same as HOME_DIR)
  uint8_t x_dir = (x_speed >= 0) ? !HOME_DIR_X : HOME_DIR_X;
  uint8_t y_dir = (y_speed >= 0) ? !HOME_DIR_Y : HOME_DIR_Y;
  uint8_t z_dir = (z_speed >= 0) ? !HOME_DIR_Z : HOME_DIR_Z;

  digitalWrite(DIR_PIN_X, x_dir);
  digitalWrite(DIR_PIN_Y, y_dir);
  digitalWrite(DIR_PIN_Z, z_dir);

  // Get absolute speeds
  float ax = x_speed >= 0 ? x_speed : -x_speed;
  float ay = y_speed >= 0 ? y_speed : -y_speed;
  float az = z_speed >= 0 ? z_speed : -z_speed;

  unsigned long current_time = micros();
  bool moved = false;

  if (ax > 0.0f) {
    unsigned long x_delay = BASE_STEP_DELAY - (ax * (BASE_STEP_DELAY - MIN_STEP_DELAY));
    if ((current_time - last_x_step) >= x_delay && canMoveX(x_dir)) {
      x_steps += (x_speed >= 0) ? 1 : -1;
      digitalWrite(STEP_PIN_X, HIGH);
      delayMicroseconds(1);
      digitalWrite(STEP_PIN_X, LOW);
      last_x_step = current_time;
      moved = true;
    }
  }

  if (ay > 0.0f) {
    unsigned long y_delay = BASE_STEP_DELAY - (ay * (BASE_STEP_DELAY - MIN_STEP_DELAY));
    if ((current_time - last_y_step) >= y_delay && canMoveY(y_dir)) {
      y_steps += (y_speed >= 0) ? 1 : -1;
      digitalWrite(STEP_PIN_Y, HIGH);
      delayMicroseconds(1);
      digitalWrite(STEP_PIN_Y, LOW);
      last_y_step = current_time;
      moved = true;
    }
  }

  if (az > 0.0f) {
    unsigned long z_delay = BASE_STEP_DELAY - (az * (BASE_STEP_DELAY - MIN_STEP_DELAY));
    if ((current_time - last_z_step) >= z_delay && canMoveZ(z_dir)) {
      z_steps += (z_speed >= 0) ? 1 : -1;
      digitalWrite(STEP_PIN_Z, HIGH);
      delayMicroseconds(1);
      digitalWrite(STEP_PIN_Z, LOW);
      last_z_step = current_time;
      moved = true;
    }
  }

  if (moved) {
    stepsUntilUpdate--;
    if (stepsUntilUpdate <= 0) {
      sendPosition();
      stepsUntilUpdate = POSITION_UPDATE_INTERVAL;
    }
  }
}

void moveToPosition(float target_x_mm, float target_y_mm, float target_z_mm) {
  long target_x_steps = mmToSteps(target_x_mm);
  long target_y_steps = mmToSteps(target_y_mm);
  long target_z_steps = mmToSteps(target_z_mm);
  
  while (x_steps != target_x_steps || y_steps != target_y_steps || z_steps != target_z_steps) {
    // Move positive (away from endstop) = opposite of HOME_DIR
    // Move negative (towards endstop) = same as HOME_DIR
    if (x_steps < target_x_steps) {
      digitalWrite(DIR_PIN_X, !HOME_DIR_X);  // Move positive
      digitalWrite(STEP_PIN_X, HIGH);
      x_steps++;
    } else if (x_steps > target_x_steps) {
      digitalWrite(DIR_PIN_X, HOME_DIR_X);   // Move negative
      digitalWrite(STEP_PIN_X, HIGH);
      x_steps--;
    }
    
    if (y_steps < target_y_steps) {
      digitalWrite(DIR_PIN_Y, !HOME_DIR_Y);  // Move positive
      digitalWrite(STEP_PIN_Y, HIGH);
      y_steps++;
    } else if (y_steps > target_y_steps) {
      digitalWrite(DIR_PIN_Y, HOME_DIR_Y);   // Move negative
      digitalWrite(STEP_PIN_Y, HIGH);
      y_steps--;
    }
    
    if (z_steps < target_z_steps) {
      digitalWrite(DIR_PIN_Z, !HOME_DIR_Z);  // Move positive
      digitalWrite(STEP_PIN_Z, HIGH);
      z_steps++;
    } else if (z_steps > target_z_steps) {
      digitalWrite(DIR_PIN_Z, HOME_DIR_Z);   // Move negative
      digitalWrite(STEP_PIN_Z, HIGH);
      z_steps--;
    }
    
    delayMicroseconds(1);
    digitalWrite(STEP_PIN_X, LOW);
    digitalWrite(STEP_PIN_Y, LOW);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(HOMING_SPEED);
    
    stepsUntilUpdate--;
    if (stepsUntilUpdate <= 0) {
      sendPosition();
      stepsUntilUpdate = POSITION_UPDATE_INTERVAL;
    }
  }
  sendPosition();
}

void sendHomingComplete() {
  Serial.write(MSG_START);
  Serial.write(MSG_TYPE_HOMING_COMPLETE);
  Serial.write(MSG_END);
}

void performHoming() {
  is_homing = true;
  
  // Max steps for each axis = 2 * max position (full axis length)
  long max_y_steps = mmToSteps(2 * MAX_Y_POSITION_MM);
  long max_x_steps = mmToSteps(2 * MAX_X_POSITION_MM);
  long max_z_steps = mmToSteps(2 * MAX_Z_POSITION_MM);
  long step_count;
  
  // Home Y axis first
  digitalWrite(DIR_PIN_Y, HOME_DIR_Y);
  bool y_left_homed = false;
  bool y_right_homed = false;
  step_count = 0;
  
  while ((!y_left_homed || !y_right_homed) && step_count < max_y_steps) {
    digitalWrite(STEP_PIN_Y, HIGH);
    delayMicroseconds(1);
    digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(HOMING_SPEED);
    step_count++;
    
    if (digitalRead(LIMIT_SWITCH_Y_L) == HIGH) y_left_homed = true;
    if (digitalRead(LIMIT_SWITCH_Y_R) == HIGH) y_right_homed = true;
  }
  
  delay(500);
  
  // Home X axis
  digitalWrite(DIR_PIN_X, HOME_DIR_X);
  step_count = 0;
  
  while (!isXEndstopHit() && step_count < max_x_steps) {
    digitalWrite(STEP_PIN_X, HIGH);
    delayMicroseconds(1);
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(HOMING_SPEED);
    step_count++;
  }
  
  delay(500);
  
  // Home Z axis
  digitalWrite(DIR_PIN_Z, HOME_DIR_Z);
  step_count = 0;
  
  while (!isZEndstopHit() && step_count < max_z_steps) {
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(1);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(HOMING_SPEED);
    step_count++;
  }
  
  // Reset positions to endstop positions
  x_steps = mmToSteps(X_ENDSTOP_POS);
  y_steps = mmToSteps(Y_ENDSTOP_POS);
  z_steps = mmToSteps(Z_ENDSTOP_POS);
  
  // Move to center position
  moveToPosition(0, 0, 0);
  
  is_homing = false;
  sendHomingComplete();  // Send properly framed homing complete message
  sendPosition();        // Then send position update
}

void processCommand(uint8_t cmd) {
  if (cmd == HOME_COMMAND) {
    performHoming();
    return;
  }

  if (cmd == CMD_SET_VELOCITY) {
    // Extract signed velocities
    union { float f; uint8_t b[4]; } x_conv, y_conv, z_conv;
    memcpy(x_conv.b, &receiveBuffer[2], 4);
    memcpy(y_conv.b, &receiveBuffer[6], 4);
    memcpy(z_conv.b, &receiveBuffer[10], 4);
    x_speed = x_conv.f;
    y_speed = y_conv.f;
    z_speed = z_conv.f;
    return;
  }
}

void loop() {
  // Handle incoming messages
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    if (inByte == MSG_START) {
      messageStarted = true;
      receiveIndex = 0;
      continue;
    }
    if (messageStarted) {
      if (inByte == MSG_END) {
        if (receiveIndex >= 14 && receiveBuffer[0] == MSG_TYPE_COMMAND) {
          processCommand(receiveBuffer[1]);
        }
        messageStarted = false;
      } else if (receiveIndex < sizeof(receiveBuffer)) {
        receiveBuffer[receiveIndex++] = inByte;
      } else {
        messageStarted = false;
      }
    }
  }

  // Generate steps continuously based on current velocities
  stepFromVelocities();
}