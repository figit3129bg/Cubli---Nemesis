#include "imu_bmi270.h"
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <cmath>

namespace {
BMI270 imu;
constexpr uint8_t kI2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68
float theta_b_dotX, theta_b_dotY, theta_b_dotZ, theta_b_accel;
}  // namespace

float theta_b_dot = 0.0f;
float theta_b = 0.0f;

void imuSetup()
{
    Serial.println("BMI270 Example 1 - Basic Readings I2C");
    Wire.begin();
    while (imu.beginI2C(kI2cAddress) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }
    Serial.println("BMI270 connected!");
}

void imuLoop()
{
    imu.getSensorData();

    theta_b_dotX = imu.data.gyroX;
    theta_b_dotY = imu.data.gyroY;
    theta_b_dotZ = imu.data.gyroZ;

    theta_b_dot = imu.data.gyroY * PI/180.0; //change depending on the axes

    theta_b_accel = atan2( imu.data.accelX , imu.data.accelZ) ; //in order to know the axes you have to determine how the IMU is mounted

    theta_b = theta_b_accel;
}
