#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "IMUYawTracker.h"
#include <cmath>
#include <math.h>

// Defining Components ~ I can add timer if I want to
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// Motor A connections
#define enA 13
#define in1 14
// Motor B connections
#define enB 22
#define in2 21
// Switch Connection
#define SWITCH_PIN 6
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1

// ~ Motor Interrupt Declaration Components
const byte MOTOR_A = 15;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 23;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

unsigned long lastT;
unsigned long thisT;

// Objects ~ I can add Touch Sensor later...
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);



// Variables
float mod_gyro_angle = 0;
float stepcount = 275.00;
volatile int counter_A = 0;
volatile int counter_B = 0;
float mod_angle = 0;
float speed = 0;
float e = 0;
float lastError = 0;
float Gain = 0;
float gain = 0;
float target = 0;
float correction = 0;
float powerLeft = 0;
float powerRight = 0;
int final_change = 0;
float normal = 0;
float circumference = 20.00;
float distovertime = 0;
float powerneeded = 0;
float counts = 0;
float total_e = 0;
float expected_angle = 0;
float thecorrection = 0;
float erroring_angle = 0;
float modError = 0;
float leftError = 0;
float rightError = 0;
float halfTarget = 0;
float yaw = 0.0;





void ISR_countA()  {

  // Counts for Encoder Motor A
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  {

  // Counts for Encoder Motor B
  counter_B++;  // increment Motor B counter value
}






float modErrorDetect() {
  computeYaw();
  yaw = computeYaw();
  float errorInitial = yaw;
  return errorInitial;
}






void PD(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  resetYaw();
  yaw = resetYaw();
  target = yaw;
  while ((thesteps * 2) > counter_A + counter_B) {
    computeYaw();
    yaw = computeYaw(); // look at
    thecorrection = target - float(yaw);
    powerLeft = power - (gain * thecorrection);
    powerRight = power + (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
    } 
  stop();
}

void DecelPD(int thesteps, int speedPWM) {

  counter_A = 0;
  counter_B = 0;

  resetYaw();
  yaw = resetYaw();
  target = yaw;

  int totalCounts = thesteps * 2;
  float decelStart = totalCounts * 0.70;
  float finalPWM = 180;   // decel target (slower)

  float gain = 4.99 
             + 0.0501 * speedPWM
             - 5.33e-4 * speedPWM * speedPWM
             + 1.39e-6 * speedPWM * speedPWM * speedPWM;

  while ((counter_A + counter_B) < totalCounts) {

    computeYaw();
    yaw = computeYaw();
    float thecorrection = target - float(yaw);

    int progress = counter_A + counter_B;

    // -----------------------------
    // Base speed (before decel)
    // -----------------------------
    float basePWM = speedPWM;

    // -----------------------------
    // Deceleration logic (70% → 100%)
    // -----------------------------
    if (progress >= decelStart) {

      float ratio = float(progress - decelStart) / float(totalCounts - decelStart);
      ratio = constrain(ratio, 0.0, 1.0);

      // increase PWM toward slower value
      basePWM = speedPWM + ratio * (finalPWM - speedPWM);
    }

    // -----------------------------
    // PD correction
    // -----------------------------
    float powerLeft  = basePWM - (gain * thecorrection);
    float powerRight = basePWM + (gain * thecorrection);

    // -----------------------------
    // Clamp (inverted logic safe)
    // -----------------------------
    powerLeft  = constrain(powerLeft,  0, 255);
    powerRight = constrain(powerRight, 0, 255);

    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  stop();
}








void stop() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void setup() { 
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Calibration Start"));
  display.display();
  setupSensor(CALIBRATION_DURATION);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Calibration Done"));
  display.display();
  delay(1000);
  modError = float(modErrorDetect());
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING); // Attach interupts for encoders
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  stop();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Welcome Shiven!"));
  display.println(F("Wait for Gyro..."));
  display.display();
  delay(2000); 
  while (digitalRead(SWITCH_PIN) == LOW) {
    computeYaw();
    yaw = computeYaw();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.display();
  }
  delay(250);
  computeYaw();
  yaw = computeYaw();
  halfTarget = float(yaw);
  // PD(406,135,5.5);
  DecelPD(406,135);
  
  

  
  


  
}

/*

display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.print(" CountA: ");
    display.println(counter_A);
    display.print(" CountB: ");
    display.println(counter_B);
    display.display();

*/
void loop() {
  /*
   Basic code to run repeatedly
  */
}


