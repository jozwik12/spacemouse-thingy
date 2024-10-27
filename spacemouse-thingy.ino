#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

void setup(void) {
  // Turn on serial for debugging
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Initialize
  Serial.println("Initializing");
  if (!icm.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
}

float angleX = 0, angleY = 0, angleZ = 0;
float prevAngleX = 0, prevAngleY = 0, prevAngleZ = 0;
float gyroThreshold = 0.02;

float degreeConv = 57.3;
unsigned long lastUpdateTime = 0; // to store the last time variables were updated
const unsigned long timeout = 5000; // 5 seconds timeout

float alpha = 0.95; // Adjust alpha based on your system dynamics

void loop() {

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // if (angleX != prevAngleX || angleY != prevAngleY || angleZ != prevAngleZ) {
  //   lastUpdateTime = millis(); // Update the time when variables change
  //   prevAngleX = angleX; // Update previous values
  //   prevAngleY = angleY;
  //   prevAngleZ = angleZ;
  // }

  // // Check if 5 seconds have passed since the last change
  // if (millis() - lastUpdateTime > timeout) {
  //   angleX = 0;
  //   angleY = 0;
  //   angleZ = 0;
  //   prevAngleX = 0; // Reset previous values
  //   prevAngleY = 0;
  //   prevAngleZ = 0;
  //   lastUpdateTime = millis(); // Reset the timer to avoid multiple resets
  // }

  if(abs(gyro.gyro.x) > gyroThreshold) {
    angleX += gyro.gyro.x * measurement_delay_us * degreeConv / 1000000;
  }
  if(abs(gyro.gyro.y) > gyroThreshold) {
    angleY += gyro.gyro.y * measurement_delay_us * degreeConv / 1000000;
  }
  if(abs(gyro.gyro.z) > gyroThreshold) {
    angleZ += gyro.gyro.z * measurement_delay_us * degreeConv / 1000000;
  }

  // Serial.print("\tMag X: ");
  // Serial.print(mag.magnetic.x);
  // Serial.print(" \tY: ");
  // Serial.print(mag.magnetic.y);
  // Serial.print(" \tZ: ");
  // Serial.print(mag.magnetic.z);
  // Serial.print(" uT");

  // Serial.print("\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.print(" rad/s ");

  Serial.print("\t\tAng X: ");
  Serial.print(angleX);
  Serial.print(" \tY: ");
  Serial.print(angleY);
  Serial.print(" \tZ: ");
  Serial.print(angleZ);
  Serial.print(" deg ");

  Serial.println();
  
  delayMicroseconds(measurement_delay_us);
}
