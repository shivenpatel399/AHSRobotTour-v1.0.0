// === Hardware Definitions ===

// Motor A connections
#define enA 13
#define in1 14

// Motor B connections
#define enB 37
#define in2 36

// Switch Connection
#define SWITCH_PIN 41

// Motor Interrupt Declaration Components
const byte MOTOR_A = 15;  // Right Motor (not used here)
const byte MOTOR_B = 23;  // Left Motor (not used here)


// === State Management ===
bool motorsRunning = false;     // Track if motors are active
bool hasRunOnce = false;        // Ensures it only runs once total
unsigned long motorStartTime = 0;
const unsigned long motorRunDuration = 2000;  // milliseconds
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 50;       // milliseconds


void setup() {
  Serial.begin(9600);

  // Pin modes
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP); // Button to GND

  // Stop motors initially (255 = stop)
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  Serial.println("Ready. Press the switch to start motors (runs only once).");
}


void loop() {
  unsigned long currentTime = millis();
  bool buttonState = digitalRead(SWITCH_PIN) == LOW; // Active LOW

  // === Debounce & Button Detection ===
  static bool lastButtonState = HIGH;
  if (buttonState != lastButtonState) {
    lastButtonPress = currentTime;
  }

  // Check for valid button press (and if not already run)
  if ((currentTime - lastButtonPress) > debounceDelay && 
      buttonState == LOW && 
      lastButtonState == HIGH && 
      !hasRunOnce) {
        
    if (!motorsRunning) {
      startMotors();
      motorStartTime = currentTime;
      motorsRunning = true;
      hasRunOnce = true;  // Mark as done forever
    }
  }

  lastButtonState = buttonState;

  // === Motor Run Timing ===
  if (motorsRunning && (currentTime - motorStartTime >= motorRunDuration)) {
    stopMotors();
    motorsRunning = false;
    Serial.println("Motors stopped. Program will not run again.");
  }
}


void startMotors() {
  Serial.println("Motors starting...");
  analogWrite(enA, 0);   // 0 = full speed (inverted)
  analogWrite(enB, 0);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
}

void stopMotors() {
  Serial.println("Motors stopping...");
  analogWrite(enA, 255); // 255 = stop
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
