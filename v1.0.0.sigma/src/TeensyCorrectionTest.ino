#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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
Adafruit_LSM6DSOX sox;


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


unsigned long lastTime;
unsigned long thistime;
unsigned long initialTime;
unsigned long currentTime;
unsigned long OVlasttime;
unsigned long OVthistime;
unsigned long lastT;
unsigned long thisT;

float gyroZ_offset = 0;
float yaw_angle = 0;
const float STRAIGHT_THRESHOLD = 0.5;
const unsigned long STRAIGHT_MIN_TIME = 100;
unsigned long straightStartTime = 0;
bool wasStraight = false;

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

void initialCalibrateGyroZ(unsigned long calibDurationMs = 2000) {
  unsigned long start = millis();
  float sum = 0;
  int count = 0;
  sensors_event_t accel, gyro, temp;

  while (millis() - start < calibDurationMs) {
    sox.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z * SENSORS_RADS_TO_DPS;
    count++;
    delay(2);
  }
  if (count > 0) {
    gyroZ_offset = sum / count;
  }
}

void updateYawAngle() {
  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);

  float curr_gyrZ = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (fabs(curr_gyrZ) < STRAIGHT_THRESHOLD) {
    if (!wasStraight) {
      straightStartTime = now;
      wasStraight = true;
    }
    if (now - straightStartTime > STRAIGHT_MIN_TIME) {
      gyroZ_offset = 0.99 * gyroZ_offset + 0.01 * curr_gyrZ;
    }
  } else {
    wasStraight = false;
  }

  float yaw_rate = curr_gyrZ - gyroZ_offset;
  yaw_angle += yaw_rate * dt;

  // Normalize yaw to -360 → +360
  yaw_angle = fmod(yaw_angle, 360.0);
  if (yaw_angle > 360.0) yaw_angle -= 360.0;
  if (yaw_angle < -360.0) yaw_angle += 360.0;
}

float modErrorDetect() {
  updateYawAngle();
  float errorInitial = yaw_angle;
  return errorInitial;
}

void subLeft(float speed) { // 32 is official speed with 85 degrees
  updateYawAngle();
  float angle = yaw_angle;
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  updateYawAngle();
  angle = yaw_angle;
  expected_angle = mod_gyro_angle + 90;
  while (float(yaw_angle) <  expected_angle) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    updateYawAngle();
  }
  analogWrite(enA, 0);
  stop();
  
}

void subRight(float speed) { // 32 is official speed with 85 degrees
  updateYawAngle();
  float angle = yaw_angle;
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  updateYawAngle(); // focus
  angle = yaw_angle;
  expected_angle = mod_gyro_angle - 90;
  lastT = micros();
  while (float(yaw_angle) >  expected_angle) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    updateYawAngle();
  }
  thisT = micros();
  stop();
  
}

void PD(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  updateYawAngle(); // look at
  mock_mod_angle(float(yaw_angle));
  target = mod_gyro_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    updateYawAngle(); // look at
    thecorrection = target - float(yaw_angle);
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
    display.println(float(yaw_angle));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.print(" Tine: ");
    display.println(thisT-lastT);
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
    powerRight = power; // 255 - (0.601 * (255 - powerLeft));
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

void PDBACK(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  updateYawAngle(); // look at
  mock_mod_angle(float(yaw_angle));
  target = mod_gyro_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    updateYawAngle(); // look at
    thecorrection = target - float(yaw_angle);
    powerLeft = power - (gain * thecorrection);
    powerRight = power + (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw_angle));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.display();
    } 
  stop();
}

void PDTIME(double seconds, int power, float gain) {
  updateYawAngle(); // look at
  mock_mod_angle(float(yaw_angle));
  target = mod_gyro_angle;
  initialTime = micros();
  while (int(seconds * 1000000) > int(currentTime - initialTime)) {
    updateYawAngle(); // look at
    thecorrection = target - float(yaw_angle);
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
    display.println(float(yaw_angle));
    display.print(" PLeft: ");
    display.println(float(powerLeft));
    display.print(" PRight: ");
    display.println(float(powerRight));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.print(" Time: ");
    display.println(currentTime-initialTime);
    display.display();
    currentTime = micros();
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
  Serial.begin(9600);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  if (!sox.begin_I2C()) {
    while (1) delay(10);
  }
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  initialCalibrateGyroZ(2000);
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
    updateYawAngle();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(yaw_angle));
    display.display();
  }
  delay(250);
  updateYawAngle();
  halfTarget = float(yaw_angle);
  straightTime(2000000,150);



  
}

void loop() {
  /*
   Basic code to run repeatedly
  */
  
}


