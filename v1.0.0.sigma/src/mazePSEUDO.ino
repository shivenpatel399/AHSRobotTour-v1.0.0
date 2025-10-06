#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define enA 13
#define in1 11
#define in2 12
#define enB 8
#define in3 10
#define in4 9
#define OLED_RESET -1

const byte MOTOR_A = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
Adafruit_LSM6DSOX sox;

float gyroZ_offset = 0;
float yaw_angle = 0;
unsigned long lastTime = 0;
const float STRAIGHT_THRESHOLD = 0.5;
const unsigned long STRAIGHT_MIN_TIME = 100;
unsigned long straightStartTime = 0;
bool wasStraight = false;
float expected_angle;

float mod_gyro_angle = 0;
float stepcount = 275.00;
volatile int counter_A = 0;
volatile int counter_B = 0;
String Position = "North";
float mod_angle = 0;
int Turns = 0;
float Straights = 0;
bool moving = false;
float remaining_seconds = 0;
float total_distance = 0;
float speed = 0;
float e = 0;
float integral = 0;
float derivative = 0;
float lastError = 0;
double kpGain = 0;
double kiGain = 0;

float SENSORS_RADS_TO_DPS = 57.295779513;

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
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastTime = currentTime;

  float yaw_rate = curr_gyrZ - gyroZ_offset;
  yaw_angle += yaw_rate * dt;

  yaw_angle = fmod(yaw_angle + 180.0, 360.0);
  if (yaw_angle < 0) yaw_angle += 360.0;
  yaw_angle -= 180.0;
}

void startRobot() {
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  moving = true;
}

void gyro_calibration() {
  if (!sox.begin_I2C()) {
    Serial.println("LSM6DSOX not found");
  }
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  initialCalibrateGyroZ(2000);
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  moving = false;
}

void PD() {
  updateYawAngle();
  float error = expected_angle - yaw_angle;
  float P = kpGain * error;
  integral += error;
  float D = derivative * (error - lastError);
  lastError = error;

  int motorSpeedA = 200 + P;
  int motorSpeedB = 200 - P;

  analogWrite(enA, constrain(motorSpeedA, 0, 255));
  analogWrite(enB, constrain(motorSpeedB, 0, 255));
}

void PDBACK() {
  updateYawAngle();
  float error = expected_angle - yaw_angle;
  float P = kpGain * error;
  integral += error;
  float D = derivative * (error - lastError);
  lastError = error;

  int motorSpeedA = 200 - P;
  int motorSpeedB = 200 + P;

  analogWrite(enA, constrain(motorSpeedA, 0, 255));
  analogWrite(enB, constrain(motorSpeedB, 0, 255));
}

void halfstep() {
  // Example halfstep movement
  counter_A = 0;
  counter_B = 0;
  while (counter_A < stepcount / 2 && counter_B < stepcount / 2) {
    PD();
  }
  stop();
}

void laststep() {
  counter_A = 0;
  counter_B = 0;
  while (counter_A < stepcount && counter_B < stepcount) {
    PD();
  }
  stop();
}

void ISR_countA() {
  counter_A++;
}

void ISR_countB() {
  counter_B++;
}

void mock_mod_angle() {
  mod_angle = fmod(yaw_angle, 360.0);
}

void right() {
  updateYawAngle();
  expected_angle = yaw_angle + 90;
  while (fabs(yaw_angle - expected_angle) > 2) {
    PD();
    updateYawAngle();
  }
  stop();
}

void left() {
  updateYawAngle();
  expected_angle = yaw_angle - 90;
  while (fabs(yaw_angle - expected_angle) > 2) {
    PD();
    updateYawAngle();
  }
  stop();
}

void subLeft() {
  left();
}

void subRight() {
  right();
}

void north() {
  expected_angle = 0;
}

void south() {
  expected_angle = 180;
}

void east() {
  expected_angle = 90;
}

void west() {
  expected_angle = -90;
}

void speedTime_control() {
  total_distance = (counter_A + counter_B) / 2.0;
  speed = total_distance / (millis() / 1000.0);
}

void executeRobot() {
  startRobot();
  delay(1000);
  halfstep();
  right();
  laststep();
  stop();
}

void setup() {
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  Serial.begin(115200);

  if (!sox.begin_I2C()) {
    Serial.println(F("Failed to find LSM6DSOX sensor"));
    for (;;) {}
  } else {
    sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
    initialCalibrateGyroZ(2000);
  }

  attachInterrupt(digitalPinToInterrupt(MOTOR_A), ISR_countA, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_B), ISR_countB, RISING);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {}
  }
  display.display();
}

void loop() {
  executeRobot();
  delay(2000);
}
