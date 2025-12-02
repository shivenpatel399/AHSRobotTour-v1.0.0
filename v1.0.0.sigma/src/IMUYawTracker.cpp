#include "IMUYawTracker.h"

// ========== Config/Globals ===========
SensorType selectedSensor = SENSOR_BMI088_I2C; // <-- Change here if needed
GyroRangeType selectedGyroRange = GYRO_RANGE_250; // <-- Change if needed

const unsigned long CALIBRATION_DURATION = 4000;
const float STRAIGHT_THRESHOLD = 0.5;      // deg/s
const unsigned long STRAIGHT_MIN_TIME = 100; // ms

Bmi088Accel* accel = nullptr;
Bmi088Gyro*  gyro  = nullptr;
Adafruit_LSM6DSOX sox;

float gyroZ_offset = 0;
float yaw_angle = 0;
unsigned long lastTime = 0;
unsigned long straightStartTime = 0;
bool wasStraight = false;

#define RAD_TO_DEG 57.2957795

float getGyroZ_degps() {
  if (selectedSensor == SENSOR_BMI088_I2C || selectedSensor == SENSOR_BMI088_SPI) {
    gyro->readSensor();
    return gyro->getGyroZ_rads() * RAD_TO_DEG;
  } else if (selectedSensor == SENSOR_LSM6DSOX) {
    sensors_event_t accel_ev, gyro_ev, temp_ev;
    sox.getEvent(&accel_ev, &gyro_ev, &temp_ev);
    return gyro_ev.gyro.z * SENSORS_RADS_TO_DPS;
  }
  return 0.0;
}

void setupSensor(unsigned long calibDurationMs) {
  if (selectedSensor == SENSOR_BMI088_I2C) {
    Wire.begin();
    Wire.setClock(400000);
    gyro  = new Bmi088Gyro(Wire, BMI088_GYRO_ADDR);
    Serial.println("BMI088 using I2C.");
  } else if (selectedSensor == SENSOR_BMI088_SPI) {
    // Set up CS pins for SPI
    pinMode(GYRO_CS_PIN, OUTPUT);
    digitalWrite(GYRO_CS_PIN, HIGH);  // Deselect
    SPI.begin();
    gyro  = new Bmi088Gyro(SPI, GYRO_CS_PIN);
    Serial.print("BMI088 using SPI. CSB2 (Gyro): ");
    Serial.println(GYRO_CS_PIN);
  }

  if (selectedSensor == SENSOR_BMI088_I2C || selectedSensor == SENSOR_BMI088_SPI) {
    int gyroStatus = gyro->begin();
    if (gyroStatus < 0) {
      Serial.println("BMI088 initialization failed!");
      while (1) { delay(10); }
    }

    if (selectedGyroRange == GYRO_RANGE_125) {
      gyro->setRange(Bmi088Gyro::RANGE_125DPS);
      Serial.println("BMI088 Gyro set to 125 dps.");
    } else {
      gyro->setRange(Bmi088Gyro::RANGE_250DPS);
      Serial.println("BMI088 Gyro set to 250 dps.");
    }
    gyro->setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    Serial.println("BMI088 selected.");
  } else if (selectedSensor == SENSOR_LSM6DSOX) {
    Wire.begin();
    Wire.setClock(400000);
    if (!sox.begin_I2C()) {
      Serial.println("LSM6DSOX initialization failed!");
      while (1) { delay(10); }
    }

    if (selectedGyroRange == GYRO_RANGE_125) {
      sox.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
      Serial.println("LSM6DSOX Gyro set to 125 dps.");
    } else {
      sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
      Serial.println("LSM6DSOX Gyro set to 250 dps.");
    }
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("LSM6DSOX selected.");
  }

  Serial.println("Initial Gyro Z calibration: Keep sensor stationary...");
  unsigned long start = millis();
  float sum = 0;
  int count = 0;

  while (millis() - start < calibDurationMs) {
    float curr_gyrZ = getGyroZ_degps();
    sum += curr_gyrZ;
    count++;
    delay(2);
  }
  if (count > 0) {
    gyroZ_offset = sum / count;
    Serial.print("Initial Gyro Z offset: ");
    Serial.println(gyroZ_offset, 6);
  }
  lastTime = millis();
}

float computeYaw() {
  float curr_gyrZ = getGyroZ_degps();

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

  // Map yaw_angle to the 0 to 360 degree range
  yaw_angle = fmod(yaw_angle, 360.0);
  // if (yaw_angle < 0) {
  //   yaw_angle += 360.0;
  // }

  return yaw_angle;
}

float resetYaw() {
  yaw_angle = 0.0;
  lastTime = millis(); // Important: Reset time so the next integration step is accurate
  // Serial.println("--- Yaw Reset to 0 ---");
  return yaw_angle;
}
