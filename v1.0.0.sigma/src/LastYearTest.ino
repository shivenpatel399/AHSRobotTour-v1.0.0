// Libraries
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
#define in1 11
#define in2 12
// Motor B connections
#define enB 8
#define in3 10
#define in4 9
// Switch Connection
#define SWITCH_PIN 40

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1
// ~ Motor Interrupt Declaration Components
const byte MOTOR_A = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

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

unsigned long lasttime;
unsigned long thistime;
unsigned long OVlasttime;
unsigned long OVthistime;


// Functions




void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
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
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Power A: ");
    display.println(powerLeft);
    display.print(" Power B: ");
    display.println(powerRight);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.display();
    } 
  stop();
}


void decelPD(int thesteps, int power) {

  counter_A = 0;
  counter_B = 0;

  float gain = 4.61 + (-1.32e-3 * power) + (7.74e-5 * power * power);

  resetYaw();
  yaw = resetYaw();
  target = yaw;

  int totalCounts = thesteps * 2;
  float decelStart = totalCounts * 0.80; 
  float minPWM = 43;                     

  while ((counter_A + counter_B) < totalCounts) {

    computeYaw();
    yaw = computeYaw();

    float thecorrection = target - float(yaw);

    float leftPWM  = power - (gain * thecorrection);
    float rightPWM = power + (gain * thecorrection);

    int progress = counter_A + counter_B;

    if (progress >= decelStart) {

      float ratio = float(totalCounts - progress) / float(totalCounts - decelStart);
      ratio = constrain(ratio, 0.0, 1.0);

      leftPWM  = minPWM + ratio * (leftPWM  - minPWM);
      rightPWM = minPWM + ratio * (rightPWM - minPWM);
    }

    leftPWM  = constrain(leftPWM,  minPWM, 255);
    rightPWM = constrain(rightPWM, minPWM, 255);

    analogWrite(enA, leftPWM);
    analogWrite(enB, rightPWM);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Power A: ");
    display.println(leftPWM);
    display.print(" Power B: ");
    display.println(rightPWM);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.display();

  }

  stop();
  delay(60);
}


void subLeft(float speed) { // 32 is official speed with 85 degrees
  computeYaw();
  yaw = computeYaw();
  float angle = yaw;
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  computeYaw();
  yaw = computeYaw();
  angle = yaw;
  expected_angle = -88.5;
  while (float(yaw) >  expected_angle) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
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
  stop();
  delay(60);
  
}

void decelLeft(float startSpeed, float decelPercent) {
  const float targetAngle = -88.5;
  const int minPWM = 43;

  startSpeed = constrain(startSpeed, minPWM, 255);
  decelPercent = constrain(decelPercent, 0, 100);

  // Initial yaw
  yaw = computeYaw();
  float startAngle = yaw;

  float totalTurn = startAngle - targetAngle;
  float decelStartAngle = startAngle - (decelPercent / 100.0) * totalTurn;

  while (yaw > targetAngle) {

    float pwm;

    if (yaw > decelStartAngle) {
      // Full speed phase
      pwm = startSpeed;
    } else {
      // Deceleration phase
      float angleRemaining = yaw - targetAngle;
      float decelRange = decelStartAngle - targetAngle;

      pwm = minPWM + (angleRemaining / decelRange) * (startSpeed - minPWM);
    }

    pwm = constrain(pwm, minPWM, startSpeed);

    analogWrite(enA, pwm);
    analogWrite(enB, pwm);

    // Left turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    yaw = computeYaw();

    // Debug
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Angle: ");
    display.println(yaw);
    display.print("PWM: ");
    display.println(pwm);
    display.display();
  }

  stop();
  delay(60);
}




void subRight(float speed) { // 32 is official speed with 85 degrees
  computeYaw();
  yaw = computeYaw();
  float angle = yaw;
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  computeYaw();
  yaw = computeYaw(); // focus
  angle = yaw;
  expected_angle = 88.5;
  while (float(yaw) < expected_angle) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
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
  stop();
  delay(60);
  
}

void decelRight(float startSpeed, float decelPercent) {
  const float targetAngle = 88.5;
  const int minPWM = 43;

  startSpeed = constrain(startSpeed, minPWM, 255);
  decelPercent = constrain(decelPercent, 0, 100);

  // Initial yaw
  yaw = computeYaw();
  float startAngle = yaw;

  float totalTurn = targetAngle - startAngle;
  float decelStartAngle = startAngle + (decelPercent / 100.0) * totalTurn;

  while (yaw < targetAngle) {

    float pwm;

    if (yaw < decelStartAngle) {
      // Full speed phase
      pwm = startSpeed;
    } else {
      // Deceleration phase
      float angleRemaining = targetAngle - yaw;
      float decelRange = targetAngle - decelStartAngle;

      pwm = minPWM + (angleRemaining / decelRange) * (startSpeed - minPWM);
    }

    pwm = constrain(pwm, minPWM, startSpeed);

    analogWrite(enA, pwm);
    analogWrite(enB, pwm);

    // Right turn
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    yaw = computeYaw();

    // Debug
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Angle: ");
    display.println(yaw);
    display.print("PWM: ");
    display.println(pwm);
    display.display();
  }

  stop();
  delay(60);
}



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


/* 

Tasks:
*/

void setup() { 
  
  // Basic Void Setup Function for Arduino (Required to run)
  // Wire and Starting the Whole Robot in one function.
    
  Serial.begin(9600);
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
  // attachInterrupt(digitalPinToInterrupt (GYRO_INT), ISR_GYRO_READ, RISING);
  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
	// Turn off motors - Initial state
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
  decelPD(240,90); // halfstep
  //subLeft(53);
  decelLeft(53, 20);
  decelPD(431,90);
  

  // Start the Code
  
}

void loop() {
  /*
   Basic code to run repeatedly
  */
}
