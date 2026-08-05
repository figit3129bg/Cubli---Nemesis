#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <cmath>

BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

float theta_b_dot, theta_b_dotX, theta_b_dotY, theta_b_dotZ, theta_b_accel, theta_b;
float theta_offset = 0.0f; // rad, the raw atan2 reading that we want to call "0"

// keeps result in (-pi, pi] even if we subtract an offset near the +-180 seam
float wrapToPi(float angle)
{
    while (angle > PI)  angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

void calibrateZero()
{
    Serial.println("Calibrating zero reference...");
    Serial.println("Hold the cube steady at the -135 deg position now.");
    delay(3000); // time to get into position

    const int N = 200;
    double sum = 0;
    for (int i = 0; i < N; i++)
    {
        imu.getSensorData();
        float raw = atan2(imu.data.accelY, imu.data.accelX);
        sum += raw;
        delay(5);
    }
    theta_offset = sum / N;

    Serial.print("theta_offset (rad): ");
    Serial.print(theta_offset, 4);
    Serial.print("  (deg): ");
    Serial.println(theta_offset * 180.0 / PI, 2);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for serial port to open

    Serial.println("BMI270 Example 1 - Basic Readings I2C");
    Wire.begin();
    while (imu.beginI2C(i2cAddress) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }
    Serial.println("BMI270 connected!");

    calibrateZero();
}

void loop()
{
    imu.getSensorData();

    theta_b_dotX = imu.data.gyroX;
    theta_b_dotY = imu.data.gyroY;
    theta_b_dotZ = imu.data.gyroZ;

    // Rotation rate around the Z-axis (converted from deg/s to rad/s)
    theta_b_dot = imu.data.gyroZ * PI / 180.0;

    // Tilt / orientation angle in the X-Y plane around the Z-axis
    theta_b_accel = atan2(imu.data.accelY, imu.data.accelX);

    // Zero-referenced angle: 0 now means "the position you calibrated at"
    theta_b = wrapToPi(theta_b_accel - theta_offset);

    // --- CSV output for the laptop-side parser: theta_b,theta_b_dot ---
    Serial.print(theta_b, 6);
    Serial.print(",");
    Serial.println(theta_b_dot, 6);

    delay(20); // Delay ~20ms to keep the terminal easily readable
}


