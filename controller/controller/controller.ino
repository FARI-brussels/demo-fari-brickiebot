#define EN_PIN 8
#define STEP_PIN_X 2
#define DIR_PIN_X 5
#define STEP_PIN_Y 3
#define DIR_PIN_Y 6
#define STEP_PIN_Z 4
#define DIR_PIN_Z 7
#define LIMIT_SWITCH_X 10
#define LIMIT_SWITCH_Y 11
#define LIMIT_SWITCH_Z 12

// Timing control
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 10; // Send every 10ms

void setup() {
    Serial.begin(115200);
    // Increase serial buffer size
    Serial.setTimeout(1);
    
    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN_X, OUTPUT);
    pinMode(DIR_PIN_X, OUTPUT);
    pinMode(STEP_PIN_Y, OUTPUT);
    pinMode(DIR_PIN_Y, OUTPUT);
    pinMode(STEP_PIN_Z, OUTPUT);
    pinMode(DIR_PIN_Z, OUTPUT);
    
    pinMode(LIMIT_SWITCH_X, INPUT);
    pinMode(LIMIT_SWITCH_Y, INPUT);
    pinMode(LIMIT_SWITCH_Z, INPUT);
}

void loop() {
    unsigned long currentTime = millis();
    
    // Only send status at fixed intervals
    if (currentTime - lastSendTime >= SEND_INTERVAL) {
        uint8_t limit_status = 0;
        
        // Read all switches at once to minimize timing variations
        bool x_state = digitalRead(LIMIT_SWITCH_X);
        bool y_state = digitalRead(LIMIT_SWITCH_Y);
        bool z_state = digitalRead(LIMIT_SWITCH_Z);
        
        if (x_state == HIGH) limit_status |= (1 << 0);
        if (y_state == HIGH) limit_status |= (1 << 1);
        if (z_state == HIGH) limit_status |= (1 << 2);
        
        // Send with start and end markers
        Serial.write(0xFF);  // Start marker
        Serial.write(limit_status);
        Serial.write(0xFE);  // End marker
        
        lastSendTime = currentTime;
    }
    
    // Process incoming commands immediately
    if (Serial.available() >= 1) {
        uint8_t cmd = Serial.read();
        
        // Process command immediately
        processCommand(cmd);
    }
}

void processCommand(uint8_t cmd) {
    uint8_t x_step = (cmd >> 0) & 1;
    uint8_t x_dir = (cmd >> 1) & 1;
    uint8_t y_step = (cmd >> 2) & 1;
    uint8_t y_dir = (cmd >> 3) & 1;
    uint8_t z_step = (cmd >> 4) & 1;
    uint8_t z_dir = (cmd >> 5) & 1;
    
    // Set all directions first
    digitalWrite(DIR_PIN_X, x_dir ? HIGH : LOW);
    digitalWrite(DIR_PIN_Y, y_dir ? HIGH : LOW);
    digitalWrite(DIR_PIN_Z, z_dir ? HIGH : LOW);
    
    // Then handle steps
    if (x_step) {
        digitalWrite(STEP_PIN_X, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP_PIN_X, LOW);
    }
    if (y_step) {
        digitalWrite(STEP_PIN_Y, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP_PIN_Y, LOW);
    }
    if (z_step) {
        digitalWrite(STEP_PIN_Z, HIGH);
        delayMicroseconds(5);
        digitalWrite(STEP_PIN_Z, LOW);
    }
}