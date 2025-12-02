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
#define enA 2
#define in1 14
// Motor B connections
#define enB 3
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
float yaw = 0;





void ISR_countA()  {

  // Counts for Encoder Motor A
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  {

  // Counts for Encoder Motor B
  counter_B++;  // increment Motor B counter value
}

void mock_mod_angle(float degrees) {
  if (degrees > -10 && degrees < 10) {
    mod_gyro_angle = 0 + halfTarget;
  } 
  else if (degrees > 80 && degrees < 100) {
    mod_gyro_angle = 90 + halfTarget;
  }
  else if (degrees > 170 && degrees < 190) {
    mod_gyro_angle = 180 + halfTarget;
  }
  else if (degrees > 260 && degrees < 280)  {
    mod_gyro_angle = 270 + halfTarget;
  }
  else if (degrees > 350 && degrees < 370) {
    mod_gyro_angle = 360 + halfTarget;
  }
  else if (degrees < -80 && degrees > -100) {
    mod_gyro_angle = -90 + halfTarget;
  }
  else if (degrees < -170 && degrees > -190) {
    mod_gyro_angle = -180 + halfTarget;
  }
  else if (degrees < -260 && degrees > -280)  {
    mod_gyro_angle = -270 + halfTarget;
  }
  else if (degrees < -350 && degrees > -370) {
    mod_gyro_angle = -360 + halfTarget;
  }
}





float modErrorDetect() {
  computeYaw();
  yaw = computeYaw();
  float errorInitial = yaw;
  return errorInitial;
}

void subLeft(float speed) { // 32 is official speed with 85 degrees
  computeYaw();
  yaw = computeYaw();
  float angle = yaw;
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  computeYaw();
  yaw = computeYaw();
  angle = yaw;
  expected_angle = mod_gyro_angle - 87;
  while (float(yaw) >  expected_angle) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    computeYaw();
    yaw = computeYaw();
  }
  analogWrite(enA, 0);
  stop();
  
}

void subRight(float speed) { // 32 is official speed with 85 degrees
  computeYaw();
  yaw = computeYaw();
  float angle = yaw;
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  computeYaw();
  yaw = computeYaw(); // focus
  angle = yaw;
  expected_angle = mod_gyro_angle + 90;
  while (float(yaw) < expected_angle) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    computeYaw();
  yaw = computeYaw();
  }
  stop();
  
}



void PD(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  resetYaw();
  yaw = resetYaw(); // look at
  mock_mod_angle(float(yaw));
  target = yaw;
  while ((thesteps * 2) > counter_A + counter_B) {
    computeYaw();
    yaw = computeYaw(); // look at
    thecorrection = target - float(yaw);
    powerLeft = power + (gain * thecorrection);
    powerRight = power - (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
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
    display.print(" PowerLeft: ");
    display.println(powerLeft);
    display.print(" PowerRight: ");
    display.println(powerRight);
    display.display();
    } 
  stop();
}

void PDBACK(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  computeYaw();
  yaw = computeYaw(); // look at
  mock_mod_angle(float(yaw));
  target = mod_gyro_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    computeYaw();
    yaw = computeYaw(); // look at
    thecorrection = target - float(yaw);
    powerLeft = power + (gain * thecorrection);
    powerRight = power - (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
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
    display.display();
    } 
  stop();
}

void straight(int thesteps, int power) {
  counter_A = 0;
  counter_B = 0;
  while ((thesteps * 2) > counter_A + counter_B) {
    powerLeft = power;
    powerRight = power;
    analogWrite(enB, powerLeft);
    analogWrite(enA, powerRight);
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.display();
    } 
  stop();
}

void straightTime(unsigned long time, int power) {
  counter_A = 0;
  counter_B = 0;
  lastT = micros();
  thisT = micros();
  while (time > (thisT - lastT)) {
    thisT = micros();
    powerLeft = power;
    powerRight = power; // 255 - (0.851 * (255 - powerLeft));
    analogWrite(enB, powerLeft);
    analogWrite(enA, powerRight);
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.display();
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
  setupSensor(CALIBRATION_DURATION);
  modError = float(modErrorDetect());
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING); // Attach interupts for encoders
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
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
  PD(1200,55,5.5);
  
  
  


  
}

void loop() {
  /*
   Basic code to run repeatedly
  */
}


