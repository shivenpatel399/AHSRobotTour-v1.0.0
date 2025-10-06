#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


#define OLED_RESET -1


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
Adafruit_LSM6DSOX sox;

float gyroZ_offset = 0;
float yaw_angle = 0;
unsigned long lastTime = 0;
const float STRAIGHT_THRESHOLD = 0.5;
const unsigned long STRAIGHT_MIN_TIME = 100;
unsigned long straightStartTime = 0;
bool wasStraight = false;



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


  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);
  }


  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Ready!"));
  display.println(F("Press switch..."));
  display.display();

  // Wait for switch press

 


}

void loop() {
  // Empty loop
  updateYawAngle();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(yaw_angle);
  display.display();
}
