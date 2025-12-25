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

// void mock_mod_angle(float degrees) {
//   if (degrees > -15 && degrees < 15) {
//     mod_gyro_angle = 0 + halfTarget;
//   } 
//   else if (degrees > 75 && degrees < 105) {
//     mod_gyro_angle = 90 + halfTarget;
//   }
//   else if (degrees > 165 && degrees < 195) {
//     mod_gyro_angle = 180 + halfTarget;
//   }
//   else if (degrees > 255 && degrees < 285)  {
//     mod_gyro_angle = 270 + halfTarget;
//   }
//   else if (degrees > 345 && degrees < 375) {
//     mod_gyro_angle = 360 + halfTarget;
//   }
//   else if (degrees < -75 && degrees > -105) {
//     mod_gyro_angle = -90 + halfTarget;
//   }
//   else if (degrees < -165 && degrees > -195) {
//     mod_gyro_angle = -180 + halfTarget;
//   }
//   else if (degrees < -255 && degrees > -285)  {
//     mod_gyro_angle = -270 + halfTarget;
//   }
//   else if (degrees < -345 && degrees > -375) {
//     mod_gyro_angle = -360 + halfTarget;
//   }
// }





float modErrorDetect() {
  computeYaw();
  yaw = computeYaw();
  float errorInitial = yaw;
  return errorInitial;
}



void subRightNew(int baseSpeed, int slowLimit) { // 32 is official speed with 85 degrees
  counter_A = 0;
  counter_B = 0;
  computeYaw();
  yaw = 0; 

  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);

  // target time
  float targetAngle = 88.5;  
  float slowdownRange = 30.0; 

  while (float(yaw) < targetAngle) {
    yaw = computeYaw();
    
    float degreesRemaining = abs(targetAngle - yaw);
    int currentSpeed;

    // Inverted Logic
    if (degreesRemaining < slowdownRange) {
      // Logic:
      // Angle 0 (At Target) -> slowLimit (e.g. 230 - Slowest)
      // Angle 30 (Start Braking) -> baseSpeed (e.g. 210 - Fastest)
      currentSpeed = map(degreesRemaining, 0, slowdownRange, slowLimit, baseSpeed);
    } else {
      // Far away -> Go Fast
      currentSpeed = baseSpeed;
    }
    analogWrite(enA, currentSpeed);
    analogWrite(enB, currentSpeed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.print(" CountA: ");
    display.println(counter_A);
    display.print(" CountB: ");
    display.println(counter_B);
    display.display();
  }
  stop(); 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(float(yaw));
  display.print(" CountA: ");
  display.println(counter_A);
  display.print(" CountB: ");
  display.println(counter_B);
  display.display();
  
}

void subLeftNew(int baseSpeed, int slowLimit) {
  counter_A = 0;
  counter_B = 0;
  computeYaw();
  yaw = 0; 

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // target time
  float targetAngle = -88.5;  
  float slowdownRange = 30.0; 

  while (float(yaw) > targetAngle) {
    yaw = computeYaw();
    
    float degreesRemaining = abs(targetAngle - yaw);
    int currentSpeed;

    // Inverted Logic
    if (degreesRemaining < slowdownRange) {
      // Logic:
      // Angle 0 (At Target) -> slowLimit (e.g. 230 - Slowest)
      // Angle 30 (Start Braking) -> baseSpeed (e.g. 210 - Fastest)
      currentSpeed = map(degreesRemaining, 0, slowdownRange, slowLimit, baseSpeed);
    } else {
      // Far away -> Go Fast
      currentSpeed = baseSpeed;
    }
    analogWrite(enA, currentSpeed);
    analogWrite(enB, currentSpeed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.print(" CountA: ");
    display.println(counter_A);
    display.print(" CountB: ");
    display.println(counter_B);
    display.display();
  }
  stop(); 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(float(yaw));
  display.print(" CountA: ");
  display.println(counter_A);
  display.print(" CountB: ");
  display.println(counter_B);
  display.display();

}

void subRight(float speed) { // 32 is official speed with 85 degrees
  computeYaw();
  yaw = computeYaw();
  float angle = yaw;
  // mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  computeYaw();
  yaw = computeYaw(); // focus
  angle = yaw;
  expected_angle = mod_gyro_angle + 85;
  while (float(yaw) < expected_angle) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    computeYaw();
    yaw = computeYaw();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.print(" CountA: ");
    display.println(counter_A);
    display.print(" CountB: ");
    display.println(counter_B);
    display.print(" Power: ");
    display.println(speed);
    display.display();
  }
  stop();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(float(yaw));
  display.print(" CountA: ");
  display.println(counter_A);
  display.print(" CountB: ");
  display.println(counter_B);
  display.print(" Power: ");
  display.println(speed);
  display.display();
}



void DecelAccel(int thesteps) {
  
  // ---- piecewise parameters ----
  float b = 185;     // slowest (start+end)
  float a = -60;    // decrease toward faster speed (PWM decreases to speed up)
  float c1 = 0.15;   // accelerate first 15%
  float c2 = 0.85;   // decelerate last 15%
  
  counter_A = 0;
  counter_B = 0;
  resetYaw();
  yaw = resetYaw();
  target = yaw;

  while ((thesteps * 2) > counter_A + counter_B) {

    float progress = float(counter_A + counter_B) / float(thesteps * 2);   // 0 → 1

    float basePower;

    float gain;

    // DECAY phase 0 → 0.15
    if(progress <= c1){
        float x = progress / c1; // map segment to 0→1
        basePower = b + a * (1 - cos(PI*x)) / 2; // a/2 * cos(x * PI) + (a/2 + b);
    }

    // CONSTANT PHASE
    else if(progress > c1 && progress < c2){
        basePower = b + a; // sustain fastest
    }

    // GROWTH phase 0.85 → 1
    else{
        float x = (progress - c2) / (1 - c2); // normalize to 0→1
        basePower = (b + a) + (-a) * (1 - cos(PI*x)) / 2; // a/2 * cos(x * PI) + (a/2 + b);
    }

    gain = 4.99 + 0.0501 * basePower - 5.33e-04 * basePower * basePower + 1.39e-06 * basePower * basePower * basePower;

    computeYaw();
    yaw = computeYaw();
    thecorrection = target - float(yaw);
    

    powerLeft  = basePower - (gain * thecorrection);
    powerRight = basePower + (gain * thecorrection);

    powerLeft  = constrain(powerLeft,  0, 255);
    powerRight = constrain(powerRight, 0, 255);

    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
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
  computeYaw();
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
  computeYaw();
  yaw = computeYaw();
}

void PDBACK(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  resetYaw();
  yaw = resetYaw();
  target = yaw;
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
  PD(406,165,5);
  
  
  

  
  


  
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


