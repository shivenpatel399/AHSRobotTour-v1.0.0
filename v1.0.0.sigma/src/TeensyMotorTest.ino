// Motor A connections
#define enA 13
#define in1 14
// Motor B connections
#define enB 22
#define in2 21
// Switch Connection
#define SWITCH_PIN 41

// ~ Motor Interrupt Declaration Components
const byte MOTOR_A = 15;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 23;  // Motor 1 Interrupt Pin - INT 0 - Left Motor



void setup() { 
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enB, 255);
  analogWrite(enA, 255);
  // while (digitalRead(SWITCH_PIN) == LOW) {
  //   // ok
  //   delay(10);
  // }
  delay(250);
  digitalWrite(in1, LOW);
  analogWrite(enA, 0);
  digitalWrite(in2, HIGH);
  analogWrite(enB, 0);
  delay(2000);



  
}

void loop() {
  /*
   Basic code to run repeatedly
  */
}


