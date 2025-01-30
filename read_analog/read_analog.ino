// Pin definitions
#define EN_PIN 8
#define STEP_PIN_X 2
#define DIR_PIN_X 5
#define STEP_PIN_Y 3
#define DIR_PIN_Y 6
#define LIMIT_SWITCH_X 37
#define LIMIT_SWITCH_Y_L 10
#define LIMIT_SWITCH_Y_R 11

// Homing configuration
#define HOME_DIR_X HIGH     // Set to LOW for negative direction, HIGH for positive
#define HOME_DIR_Y HIGH     // Set to LOW for negative direction, HIGH for positive
#define HOMING_SPEED 1000   // Microseconds between steps during homing
#define STEP_PULSE 5       // Microseconds for step pulse

// Machine parameters
#define STEPS_PER_REV 1600    // Standard stepper motor steps
#define GT2_CIRCUMFERENCE 40  // GT2 20T pulley: 20 teeth × 2mm pitch = 40mm
#define AXIS_LENGTH_X 750      // X axis length in mm
#define AXIS_LENGTH_Y 650      // Y axis length in mm

// Calculate steps per mm (using microstepping would multiply this)
const float STEPS_PER_MM = STEPS_PER_REV / GT2_CIRCUMFERENCE;  // 200/40 = 5 steps/mm

void setup() {
  Serial.begin(115200);
  Serial.println("Homing Test Program");
  Serial.println("Starting automatic homing...");
  
  // Configure pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(DIR_PIN_Y, OUTPUT);
  
  // Configure limit switches with pullup resistors
  pinMode(LIMIT_SWITCH_X, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Y_L, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Y_R, INPUT_PULLUP);
  
  // Enable motors
  digitalWrite(EN_PIN, LOW);
}

void moveAxis(int stepPin, int dirPin, bool direction, int steps) {
  digitalWrite(dirPin, direction);
  
  for(int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(HOMING_SPEED);
  }
}

void homeY() {
  Serial.println("Starting Y axis homing...");
  digitalWrite(DIR_PIN_Y, HOME_DIR_Y);
  
  bool y_left_homed = false;
  bool y_right_homed = false;
  
  // Continue until both endstops are triggered
  while (!y_left_homed || !y_right_homed) {
    // Read endstops
    y_left_homed = (digitalRead(LIMIT_SWITCH_Y_L) == HIGH);
    y_right_homed = (digitalRead(LIMIT_SWITCH_Y_R) == HIGH);
    
    // Print endstop states for debugging
    Serial.print("Y Left: "); Serial.print(y_left_homed);
    Serial.print(" Y Right: "); Serial.println(y_right_homed);
    
    // Step if either motor needs to move
    if (!y_left_homed || !y_right_homed) {
      digitalWrite(STEP_PIN_Y, HIGH);
      delayMicroseconds(STEP_PULSE);
      digitalWrite(STEP_PIN_Y, LOW);
      delayMicroseconds(HOMING_SPEED);
    }
  }
  
  Serial.println("Y axis homing complete");
  
  // Move to center of Y axis
  int stepsToCenter = (AXIS_LENGTH_Y / 2.0) * STEPS_PER_MM;
  moveAxis(STEP_PIN_Y, DIR_PIN_Y, !HOME_DIR_Y, stepsToCenter);
  Serial.println("Y axis centered");
}

void homeX() {
  Serial.println("Starting X axis homing...");
  digitalWrite(DIR_PIN_X, HOME_DIR_X);
  
  while (digitalRead(LIMIT_SWITCH_X) != HIGH) {
    // Print endstop state for debugging
    Serial.print("X endstop: "); 
    Serial.println(digitalRead(LIMIT_SWITCH_X));
    
    digitalWrite(STEP_PIN_X, HIGH);
    delayMicroseconds(STEP_PULSE);
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(HOMING_SPEED);
  }
  
  Serial.println("X axis homing complete");
  
  // Move to center of X axis
  int stepsToCenter = (AXIS_LENGTH_X / 2.0) * STEPS_PER_MM;
  moveAxis(STEP_PIN_X, DIR_PIN_X, !HOME_DIR_X, stepsToCenter);
  Serial.println("X axis centered");
}

void performHoming() {
  Serial.println("Starting homing sequence");
  
  // Home Y first (both motors)
  homeY();
  delay(1000); // Pause between axes
  
  // Then home X
  homeX();
  
  Serial.println("Homing and centering complete");
}

void loop() {
  // Start homing immediately after boot
  performHoming();
  
  // Wait here forever after homing is complete
  while(1) {
    delay(1000);
  }
}