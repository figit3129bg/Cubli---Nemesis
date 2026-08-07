#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <cmath>
#include "imu.h"
BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;
const float ALPHA = 0.96f;
float theta_b_dot, theta_b_dotX, theta_b_dotY, theta_b_dotZ, theta_b_accel, theta_b;
float theta_offset = 0.0f;
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
    delay(3000);
    const int N = 200;
    double sum = 0;
    for (int i = 0; i < N; i++)
    {
        imu.getSensorData();
        sum += atan2(imu.data.accelX, imu.data.accelY);
        delay(5);
    }
    theta_offset = sum / N;
    Serial.print("theta_offset (rad): ");
    Serial.print(theta_offset, 4);
    Serial.print("  (deg): ");
    Serial.println(theta_offset * 180.0 / PI, 2);
}
void imu_setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
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
void imu_loop()
{
    static uint32_t t_prev_us = 0;
    static float theta_b_prev = 0.0f;
    static bool first_run = true;
    imu.getSensorData();
    uint32_t t_now_us = micros();
    float dt = (t_now_us - t_prev_us) * 1e-6f;
    t_prev_us = t_now_us;
    theta_b_dotX = imu.data.gyroX;
    theta_b_dotY = imu.data.gyroY;
    theta_b_dotZ = imu.data.gyroZ;
    theta_b_dot = imu.data.gyroZ * PI / 180.0f;
    theta_b_accel = atan2(imu.data.accelX, imu.data.accelY);
    float z = wrapToPi(theta_b_accel - theta_offset);
    if (first_run)
    {
        theta_b_prev = z;
        first_run = false;
        return;
    }
    float theta_pred = wrapToPi(theta_b_prev + theta_b_dot * dt);
    theta_b = wrapToPi(theta_pred + (1.0f - ALPHA) * wrapToPi(z - theta_pred));
    theta_b_prev = theta_b;
    Serial.print(theta_b, 6);
    Serial.print(",");
    Serial.println(theta_b_dot, 6);
    delay(20);
}