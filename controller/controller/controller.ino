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

#define HOME_DIR_X HIGH   
#define HOME_DIR_Y HIGH    
#define HOMING_SPEED 500  
#define HOME_COMMAND 0x0F 

#define STEP_DELAY 500
#define POSITION_UPDATE_INTERVAL 100
#define BUFFER_SIZE 128

// Motion parameters
#define STEPS_PER_REV 1600
#define MM_PER_REV 40
#define MAX_X_POSITION_MM 600  // Maximum travel in millimeters
#define MAX_Y_POSITION_MM 600  // Adjust these values according to your machine

// Center position calculations
#define CENTER_X_MM (MAX_X_POSITION_MM / 2)  // Half of max X travel
#define CENTER_Y_MM (MAX_Y_POSITION_MM / 2)  // Half of max Y travel

bool is_homing = false;
long x_steps = 0;
long y_steps = 0;
long z_steps = 0;

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

bool canMoveX(bool direction) {
  if (direction && isXEndstopHit()) {
    return false;
  }
  float current_pos_mm = stepsToMM(x_steps);
  if (!direction && current_pos_mm >= MAX_X_POSITION_MM) {
    return false;
  }
  return true;
}

bool canMoveY(bool direction) {
  if (direction && isYEndstopsHit()) {
    return false;
  }
  float current_pos_mm = stepsToMM(y_steps);
  if (!direction && current_pos_mm >= MAX_Y_POSITION_MM) {
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
  
  Serial.write(0xFF);
  Serial.write(x_conv.b, 4);
  Serial.write(y_conv.b, 4);
  Serial.write(z_conv.b, 4);
  Serial.write(0xFE);
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

void executeBufferedStep(uint8_t command) {
  uint8_t x_step = (command >> 0) & 1;
  uint8_t x_dir = (command >> 1) & 1;
  uint8_t y_step = (command >> 2) & 1;
  uint8_t y_dir = (command >> 3) & 1;
  uint8_t z_step = (command >> 4) & 1;
  uint8_t z_dir = (command >> 5) & 1;
  
  bool moved = false;
  
  digitalWrite(DIR_PIN_X, x_dir ? HIGH : LOW);
  digitalWrite(DIR_PIN_Y, y_dir ? HIGH : LOW);
  digitalWrite(DIR_PIN_Z, z_dir ? HIGH : LOW);

  if (x_step && canMoveX(x_dir)) {
    x_steps += (x_dir ? -1 : 1);
    digitalWrite(STEP_PIN_X, HIGH);
    moved = true;
  }

  if (y_step && canMoveY(y_dir)) {
    y_steps += (y_dir ? -1 : 1);
    digitalWrite(STEP_PIN_Y, HIGH);
    moved = true;
  }

  if (z_step) {
    z_steps += (z_dir ? 1 : -1);
    digitalWrite(STEP_PIN_Z, HIGH);
    moved = true;
  }

  if (moved) {
    delayMicroseconds(1);
    digitalWrite(STEP_PIN_X, LOW);
    digitalWrite(STEP_PIN_Y, LOW);
    digitalWrite(STEP_PIN_Z, LOW);
    
    stepsUntilUpdate--;
    if (stepsUntilUpdate <= 0) {
      sendPosition();
      stepsUntilUpdate = POSITION_UPDATE_INTERVAL;
    }
  }
}

void moveToPosition(float target_x_mm, float target_y_mm) {
  long target_x_steps = mmToSteps(target_x_mm);
  long target_y_steps = mmToSteps(target_y_mm);
  
  while (x_steps != target_x_steps || y_steps != target_y_steps) {
    if (x_steps < target_x_steps) {
      digitalWrite(DIR_PIN_X, LOW);  // Move positive
      digitalWrite(STEP_PIN_X, HIGH);
      x_steps++;
    } else if (x_steps > target_x_steps) {
      digitalWrite(DIR_PIN_X, HIGH);  // Move negative
      digitalWrite(STEP_PIN_X, HIGH);
      x_steps--;
    }
    
    if (y_steps < target_y_steps) {
      digitalWrite(DIR_PIN_Y, LOW);  // Move positive
      digitalWrite(STEP_PIN_Y, HIGH);
      y_steps++;
    } else if (y_steps > target_y_steps) {
      digitalWrite(DIR_PIN_Y, HIGH);  // Move negative
      digitalWrite(STEP_PIN_Y, HIGH);
      y_steps--;
    }
    
    delayMicroseconds(1);
    digitalWrite(STEP_PIN_X, LOW);
    digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(HOMING_SPEED);
    
    stepsUntilUpdate--;
    if (stepsUntilUpdate <= 0) {
      sendPosition();
      stepsUntilUpdate = POSITION_UPDATE_INTERVAL;
    }
  }
  sendPosition();
}

void performHoming() {
  is_homing = true;
  
  // Home Y axis first
  digitalWrite(DIR_PIN_Y, HOME_DIR_Y);
  bool y_left_homed = false;
  bool y_right_homed = false;
  
  while (!y_left_homed || !y_right_homed) {
    if (!y_left_homed || !y_right_homed) {
      digitalWrite(STEP_PIN_Y, HIGH);
      delayMicroseconds(1);
      digitalWrite(STEP_PIN_Y, LOW);
      delayMicroseconds(HOMING_SPEED);
    }
    
    if (digitalRead(LIMIT_SWITCH_Y_L) == HIGH) y_left_homed = true;
    if (digitalRead(LIMIT_SWITCH_Y_R) == HIGH) y_right_homed = true;
  }
  
  delay(500);
  
  // Home X axis
  digitalWrite(DIR_PIN_X, HOME_DIR_X);
  while (!isXEndstopHit()) {
    digitalWrite(STEP_PIN_X, HIGH);
    delayMicroseconds(1);
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(HOMING_SPEED);
  }
  
  // Reset positions to 0
  x_steps = 0;
  y_steps = 0;
  z_steps = 0;
  
  // Move to center position
  moveToPosition(CENTER_X_MM, CENTER_Y_MM);
  
  is_homing = false;
  sendPosition();
}

void processCommand(uint8_t cmd) {
  if (cmd == HOME_COMMAND) {
    performHoming();
    return;
  }
  
  if (!is_homing) {
    addToBuffer(cmd);
  }
}

void loop() {
  while (Serial.available() >= 1 && !bufferIsFull()) {
    uint8_t cmd = Serial.read();
    processCommand(cmd);
  }
  
  if (!bufferIsEmpty()) {
    executeBufferedStep(commandBuffer[bufferTail]);
    bufferTail = (bufferTail + 1) % BUFFER_SIZE;
    delayMicroseconds(STEP_DELAY);
  }
}