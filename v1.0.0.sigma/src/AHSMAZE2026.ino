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


// Objects ~ I can add Touch Sensor later...
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);


// Variables
float mod_gyro_angle = 0;
volatile int counter_A = 0;
volatile int counter_B = 0;
String Position = "North";
float mod_angle = 0;
int Turns = 0;
float Straights = 0;
bool moving = false;
float remaining_seconds = 0;
float total_distance = 0;
float gain = 0;
float target = 0;
float powerLeft = 0;
float powerRight = 0;
int cycle = 0;
int routeSteps[] = {11,21,22,12,11}; 
float distovertime = 0;
float powerneeded = 0;
float counts = 0;
float expected_angle = 0;
float thecorrection = 0;
float modError = 0;
float overstepAngError = 0;
float overstepSpeedError = 0;
float halfTarget = 0;
float yaw = 0;

unsigned long OVlasttime;
unsigned long OVthistime;
unsigned long timeDifference_us;
float timeDifference_s;


// Functions

void startRobot() {
  
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
  


}



int overstepalgorithm(float angledifference) {
  double DegtoRad = (angledifference * (PI/180));
  double distanceoverstep = 50 * tan(DegtoRad);
  float counterpercm = 164/20;
  int countsneeded = distanceoverstep * counterpercm;
  return countsneeded;
}


int speedchangealgorithm(float error, float originalcounts, float originalSpeed) {
  float percentchange = error/originalcounts;
  float valuechange = percentchange * originalSpeed;
  int updatedSpeed = valuechange + originalSpeed;
  return updatedSpeed;
}

void PD(int thesteps, int power, float gain) {
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
    powerLeft = power - (gain * thecorrection);
    powerRight = power + (gain * thecorrection);
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





void stop() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void halfstep() { // Input the amount of steps robot encoder should take and its original power. 
  PD(70,180,4.875);

}

void laststep() { // Input the amount of steps robot encoder should take and its original power. 
  // Critical Change Point
  PDBACK(70, 180, 4.875);
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
  expected_angle = mod_gyro_angle - 80;
  while (float(yaw) >  expected_angle) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
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
  expected_angle = mod_gyro_angle + 80;
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
    display.display();
  }
  stop();
  
}

void left() {
  overstepAngError = overstepalgorithm(thecorrection);
  overstepSpeedError = speedchangealgorithm(overstepAngError,counts,powerneeded);
  subLeft(210);
  PD(counts + overstepAngError, overstepSpeedError, gain);
}

void right() {
  overstepAngError = overstepalgorithm(thecorrection);
  overstepSpeedError = speedchangealgorithm(overstepAngError,counts,powerneeded);
  subRight(210);
  PD(counts + overstepAngError, overstepSpeedError, gain);
}

void north() { // Critical Change Point and other parts

  // North Conditional 
  if (moving == false) {
    if (Position == "North") {
      Straights++;
      Position = "North";
    }
    else if (Position == "East") {
      Turns++;
      Straights++;
      Position = "North";
    }
    else if (Position == "West") {
      Turns++;
      Straights++;
      Position = "West";
    }
    else if (Position == "South") {
      Straights++;
      Position = "South";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      PD(counts, powerneeded, gain);
      Position = "North";
    }
    else if (Position == "East") {
      left();
      Position = "North";
    }
    else if (Position == "West") {
      right();
      Position = "North";
    }
    else if (Position == "South") {
      PDBACK(counts, powerneeded, gain);
      Position = "South";
    }
  }
}

void south() {

  // South Conditional
  if (moving == false) {
    if (Position == "North") {
      Straights++;
      Position = "North";
    }
    else if (Position == "East") {
      Turns++;
      Straights++;
      Position = "South";
    }
    else if (Position == "West") {
      Turns++;
      Straights++;
      Position = "South";
    }
    else if (Position == "South") {
      Straights++;
      Position = "South";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      PDBACK(counts, powerneeded, gain);
      Position = "North";
    }
    else if (Position == "East") {
      right();
      Position = "South";
    }
    else if (Position == "West") {
      left();
      Position = "South";
    }
    else if (Position == "South") {
      PD(counts, powerneeded, gain);
      Position = "South";
    }
  }
}

void east() {

  // East Conditional
  if (moving == false) {
    if (Position == "North") {
      Turns++;
      Straights++;
      Position = "East";
    }
    else if (Position == "East") {
      Straights++;
      Position = "East";
    }
    else if (Position == "West") {
      Straights++;
      Position = "West";
    }
    else if (Position == "South") {
      Turns++;
      Straights++;
      Position = "East";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      right();
      Position = "East";
    }
    else if (Position == "East") {
      PD(counts, powerneeded, gain);
      Position = "East";
    }
    else if (Position == "West") {
      PDBACK(counts, powerneeded, gain);
      Position = "West";
    }
    else if (Position == "South") {
      left();
      Position = "East";
    }
  }
}

void west() {

  // West Conditional
  if (moving == false) {
    if (Position == "North") {
      Turns++;
      Straights++;
      Position = "West";
    }
    else if (Position == "East") {
      Straights++;
      Position = "East";
    }
    else if (Position == "West") {
      Straights++;
      Position = "West";
    }
    else if (Position == "South") {
      Turns++;
      Straights++;
      Position = "West";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      left();
      Position = "West";
    }
    else if (Position == "East") {
      PDBACK(counts, powerneeded, gain);
      Position = "East";
    }
    else if (Position == "West") {
      PD(counts, powerneeded, gain);
      Position = "West";
    }
    else if (Position == "South") {
      right();
      Position = "West";
    }
  }
}

void speedTime_control(float Time) { // Critical Change Point
  
  // Speed over Time control
  remaining_seconds = Time - 2 - (Turns * 0.86) - 0.3;
  total_distance = Straights * 50;
  distovertime = total_distance/remaining_seconds;
  powerneeded = 161 + (5.74 * distovertime) - (0.312 * pow(distovertime, 2)) + (0.00337 * pow(distovertime, 3));
  counts = 450 - (0.663 * powerneeded) + (0.00325 * pow(powerneeded, 2)) - (0.00000505 * pow(powerneeded, 3));
  gain = 4.99 + (0.0501 * powerneeded) - (0.000533 * pow(powerneeded, 2)) + (0.00000139 * pow(powerneeded, 3));
 
}


void executeRobot() {
  // Actual Maze Solving Portion of the Code
  cycle = (sizeof(routeSteps)/sizeof(routeSteps[0])) - 1;
  for (int i = 0; i < cycle; i++) {
    if (routeSteps[i+1]-routeSteps[i] == 1) {
      east();
    }
    else if (routeSteps[i+1]-routeSteps[i] == -1) {
      west();
    }
    else if (routeSteps[i+1]-routeSteps[i] == 10) {
      north();
    }
    else if (routeSteps[i+1]-routeSteps[i] == -10) {
      south();
    }
  }
  moving = true;
  Position = "North";
  speedTime_control(13);
  // Add a timer start statement if I want to...
  while (digitalRead(SWITCH_PIN) == LOW) {
    computeYaw();
    yaw = computeYaw();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw));
    display.print(" Counts: ");
    display.println(counts);
    display.print(" DistOverTime: ");
    display.println(distovertime);
    display.print(" SecRemain: ");
    display.println(remaining_seconds);
    display.print(" totalDist: ");
    display.println(total_distance);
    display.print(" PWRNeeded: ");
    display.println(powerneeded);
    display.print(" Gain: ");
    display.println(gain);
    display.display();
  }
  OVlasttime = micros();
  halfstep();
  for (int j = 0; j < cycle; j++) {
    // Increase Delay for accurate angle stop, tradeoff is speed. Make it around 400-500
    // delay(300);
    if (routeSteps[j+1]-routeSteps[j] == 1) {
      east();
    }
    else if (routeSteps[j+1]-routeSteps[j] == -1) {
      west();
    }
    else if (routeSteps[j+1]-routeSteps[j] == 10) {
      north();
    }
    else if (routeSteps[j+1]-routeSteps[j] == -10) {
      south();
    }
  }
  delay(300);
  laststep();
  OVthistime = micros();
  timeDifference_us = OVthistime - OVlasttime; 
  timeDifference_s = (float)timeDifference_us / 1000000.0;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Time: ");
  display.print(timeDifference_s, 2);
  display.println(" sec");
  display.display();
}

/* 

Tasks:
*/

void setup() { 
  
  // Basic Void Setup Function for Arduino (Required to run)
  // Wire and Starting the Whole Robot in one function.
  startRobot();
  // Start the Code
  executeRobot();
  

  
  
  
  
}

void loop() {
  /*
   Basic code to run repeatedly
  */
}


/*

41 42 43 44
31 32 33 34
21 22 23 24
11 12 13 14

This is like the framework for our logic. its very logical. We will start coding the backup algo if neccesary. 

The steps list, that is where we input the coordinates the robot should go through. 11, 12, 22, 21 is not the actual path. 

differences from one value to another are calculated and input into Moves list. Move is a starting variable for the loop

here, we used step 1 to make the half step. But i think i should put it after the for loop

We have another for loop, where the actual action begins. If it detects either 1, -1, 10, or -10, it will perform one of the 4 functions with those 4 conditionals based on chacking the list and the gyro angle (for the 4 conditions)

The 4 conditionals matter here because robot position can vary (it can face 0, 90, 180, or 270 degrees based on the gyro) and if we don't use conditions, the robot will take these uneccesary moves and this will flag us a Stalling Penalty

Once one cycle is done, it adds i and removes the move that is first on the list, so it will be different for each cycle. The program will stop when i = len of Moves

Once we code speed, if everything goes as planned (Speed, Movement, Gates, Etc.), we should land on the target reaached at perfect time, covering 3 gates, giving us a score ranging 55-60 (Lowest Possible)

Thats the array shown of the map that is integrated into the Robot's "Brain"

I need to write speed formula. Its a matter of testing and recording data.


The main functions should look something like this in Python Format:



def blahblahblah():
    if angle is 0: 
        it will do this
    elif 90:
        or that
    elif 180:
        or that
    elif 270:
        and that'
These are used for up, down, left, right.
I will finish this today or tomorrow after my bot finishes charging.




To DO
Already have done: Half Distance and PID, PID Straights and Normal Straights, Turn communication, Obstacle Detection Modify,
Need to do: Final Step Modify, Speed Modify
*/
