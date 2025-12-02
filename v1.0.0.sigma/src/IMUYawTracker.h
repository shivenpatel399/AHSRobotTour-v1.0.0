#ifndef IMU_YAW_TRACKER_H
#define IMU_YAW_TRACKER_H

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include "BMI088.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// Sensor/interface selection
enum SensorType { SENSOR_BMI088_I2C, SENSOR_BMI088_SPI, SENSOR_LSM6DSOX };
extern SensorType selectedSensor;

// Gyro range selection
enum GyroRangeType { GYRO_RANGE_125, GYRO_RANGE_250 };
extern GyroRangeType selectedGyroRange;

// Calibration and bias parameters
extern const unsigned long CALIBRATION_DURATION;
extern const float STRAIGHT_THRESHOLD;
extern const unsigned long STRAIGHT_MIN_TIME;

// BMI088 I2C addresses
#define BMI088_ACCEL_ADDR 0x19
#define BMI088_GYRO_ADDR  0x69

// ----- Teensy 4.1 SPI CS pins -----
#define ACCEL_CS_PIN 40  // CSB1 -> Teensy 4.1 pin 40
#define GYRO_CS_PIN 10    // CSB2 -> Teensy 4.1 pin 0

void setupSensor(unsigned long calibDurationMs);
float computeYaw();
float resetYaw();

#endif