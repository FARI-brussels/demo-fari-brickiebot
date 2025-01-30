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

bool is_homing = false;
long x_position = 0;
long y_position = 0;
long z_position = 0;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 10;

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

void performHoming() {
 is_homing = true;
 
 digitalWrite(DIR_PIN_Y, HOME_DIR_Y);
 bool y_left_homed = false;
 bool y_right_homed = false;
 
 while (!y_left_homed || !y_right_homed) {
   if (!y_left_homed || !y_right_homed) {
     digitalWrite(STEP_PIN_Y, HIGH);
     delayMicroseconds(5);
     digitalWrite(STEP_PIN_Y, LOW);
     delayMicroseconds(HOMING_SPEED);
   }
   
   if (digitalRead(LIMIT_SWITCH_Y_L) == HIGH) y_left_homed = true;
   if (digitalRead(LIMIT_SWITCH_Y_R) == HIGH) y_right_homed = true;
 }
 
 delay(500);
 
 digitalWrite(DIR_PIN_X, HOME_DIR_X);
 while (digitalRead(LIMIT_SWITCH_X) != HIGH) {
   digitalWrite(STEP_PIN_X, HIGH);
   delayMicroseconds(5);
   digitalWrite(STEP_PIN_X, LOW);
   delayMicroseconds(HOMING_SPEED);
 }
 
 x_position = 0;
 y_position = 0;
 z_position = 0;
 is_homing = false;
}

void processCommand(uint8_t cmd) {
 if (cmd == HOME_COMMAND) {
   performHoming();
   return;
 }
 
 if (!is_homing) {
   uint8_t x_step = (cmd >> 0) & 1;
   uint8_t x_dir = (cmd >> 1) & 1;
   uint8_t y_step = (cmd >> 2) & 1;
   uint8_t y_dir = (cmd >> 3) & 1;
   uint8_t z_step = (cmd >> 4) & 1;
   uint8_t z_dir = (cmd >> 5) & 1;
   
   digitalWrite(DIR_PIN_X, x_dir ? HIGH : LOW);
   digitalWrite(DIR_PIN_Y, y_dir ? HIGH : LOW);
   digitalWrite(DIR_PIN_Z, z_dir ? HIGH : LOW);
   
   if (x_step) {
     x_position += (x_dir ? 1 : -1);
     digitalWrite(STEP_PIN_X, HIGH);
     delayMicroseconds(5);
     digitalWrite(STEP_PIN_X, LOW);
   }
   if (y_step) {
     y_position += (y_dir ? 1 : -1);
     digitalWrite(STEP_PIN_Y, HIGH);
     delayMicroseconds(5);
     digitalWrite(STEP_PIN_Y, LOW);
   }
   if (z_step) {
     z_position += (z_dir ? 1 : -1);
     digitalWrite(STEP_PIN_Z, HIGH);
     delayMicroseconds(5);
     digitalWrite(STEP_PIN_Z, LOW);
   }
 }
}

void loop() {
 unsigned long currentTime = millis();
 
 if (currentTime - lastSendTime >= SEND_INTERVAL) {
   uint8_t limit_status = 0;
   bool x_state = digitalRead(LIMIT_SWITCH_X);
   bool y_left_state = digitalRead(LIMIT_SWITCH_Y_L);
   bool y_right_state = digitalRead(LIMIT_SWITCH_Y_R);
   
   if (x_state == HIGH) limit_status |= (1 << 0);
   if (y_left_state == HIGH) limit_status |= (1 << 1);
   if (y_right_state == HIGH) limit_status |= (1 << 2);
   
   Serial.write(0xFF);
   Serial.write(limit_status);
   Serial.write(x_position);
   Serial.write(",");
   Serial.write(y_position);
   Serial.write(",");
   Serial.write(z_position);
   Serial.write(0xFE);
   
   lastSendTime = currentTime;
 }
 
 if (Serial.available() >= 1) {
   uint8_t cmd = Serial.read();
   processCommand(cmd);
 }
}