#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

BMI270 imu;

// Default I2C address (SDO pin tied to GND). If your breakout ties
// SDO to 3.3V instead, this needs to be BMI2_I2C_SEC_ADDR (0x69).
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;

void setup() {
    Serial.begin(115200);
    delay(500); // give the serial monitor a moment to attach

    // Default ESP32 devkit I2C pins: SDA=21, SCL=22.
    // Change these if your wiring is different.
    Wire.begin(21, 22);

    Serial.println("Connecting to BMI270...");
    while (imu.beginI2C(i2cAddress) != BMI2_OK) {
        Serial.println("BMI270 not responding - check wiring/address");
        delay(1000);
    }
    Serial.println("BMI270 connected.");
}

void loop() {
    imu.getSensorData();

    // CSV: accelX,accelY,accelZ,gyroX,gyroY,gyroZ
    Serial.print(imu.data.accelX, 4); Serial.print(",");
    Serial.print(imu.data.accelY, 4); Serial.print(",");
    Serial.print(imu.data.accelZ, 4); Serial.print(",");
    Serial.print(imu.data.gyroX, 4);  Serial.print(",");
    Serial.print(imu.data.gyroY, 4);  Serial.print(",");
    Serial.println(imu.data.gyroZ, 4);

    delay(20); // ~50 Hz for now, just to confirm it's alive
}