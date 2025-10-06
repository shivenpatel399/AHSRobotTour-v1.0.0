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
#define SWITCH_PIN 40

#define OLED_RESET -1
const byte MOTOR_A = 3;
const byte MOTOR_B = 2;

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

volatile int counter_A = 0;
volatile int counter_B = 0;

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

  yaw_angle = fmod(yaw_angle + 180.0, 360.0);
  if (yaw_angle < 0) yaw_angle += 360.0;
  yaw_angle -= 180.0;
}

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
  updateYawAngle();
  float target = yaw_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    updateYawAngle();
    float error = target - yaw_angle;
    float powerLeft = power - (gain * error);
    float powerRight = power + (gain * error);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(yaw_angle);
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(error);
    display.display();
  }
  stop();
}

void Straight(int thesteps, int power) {
  counter_A = 0;
  counter_B = 0;
  updateYawAngle();
  float target = yaw_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    updateYawAngle();
    float error = target - yaw_angle;
    float powerLeft = power;
    float powerRight = power;
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(yaw_angle);
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(error);
    display.print(" CountsA: ");
    display.println(counter_A);
    display.print(" CountsB: ");
    display.println(counter_B);
    
    display.display();
  }
  stop();
}

void Left(float speed) { // 32 is official speed with 85 degrees
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  updateYawAngle();
  float angle = yaw_angle;
  expected_angle = angle + 90;
  while (float(yaw_angle) <  expected_angle) {
    updateYawAngle();
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(yaw_angle);
    display.print(" Target: ");
    display.println(expected_angle);
    updateYawAngle();
  }
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
}

void ISR_countA()  { counter_A++; }
void ISR_countB()  { counter_B++; }

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
  lastTime = millis();

  attachInterrupt(digitalPinToInterrupt(MOTOR_A), ISR_countA, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_B), ISR_countB, RISING);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);
  }
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Ready!"));
  display.println(F("Press switch..."));
  display.display();

  // Wait for switch press
  while (digitalRead(SWITCH_PIN) == LOW) {
    delay(10);
  }

  // Calculate encoder counts for 100 cm
  int counts_100cm = 880; // 176/20*100
  int counts_50cm = 440; // 176/20*50

  int power = 70;       // Example value, adjust as needed
  float gain = 4.5;     // Example value, adjust as needed

  unsigned long startTime = millis();
  Left(45);
  // PD(counts_50cm, 80, 5);
  unsigned long endTime = millis();


  while (1); // Stay here forever
}

void loop() {
  // Empty loop
}