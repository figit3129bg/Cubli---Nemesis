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
    delay(2000);
    const int N = 500;
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
    delay(100);
}
void imu_setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("BMI270 Example 1 - Basic Readings I2C");
    Wire.begin();
    // BMI270 supports I2C Fast Mode (400kHz); Wire's default is the slower
    // 100kHz standard mode, which needlessly stretches every getSensorData()
    // transaction. Bump it up so the I2C bus isn't the bottleneck below.
    Wire.setClock(400000);
    while (imu.beginI2C(i2cAddress) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }
    Serial.println("BMI270 connected!");

    // begin() leaves the sensor at its power-on-reset output data rate
    // (100Hz for both accel and gyro), which is slower than the control
    // loop wants to run. Raise it to the max available in performance mode.
    // .range is read back and left untouched so BMI270::begin()'s already-
    // computed raw-to-physical-unit scale factors (rawToGs/rawToDegSec)
    // stay valid -- only .odr/.filter_perf/.noise_perf change here.
    bmi2_sens_config accelConfig;
    accelConfig.type = BMI2_ACCEL;
    imu.getConfig(&accelConfig);
    accelConfig.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    int8_t err = imu.setConfig(accelConfig);

    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    imu.getConfig(&gyroConfig);
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_3200HZ;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err |= imu.setConfig(gyroConfig);

    if (err != BMI2_OK)
    {
        Serial.print("Warning: high-ODR IMU config rejected, error ");
        Serial.println(err);
    }

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

    //remember to change for 3D
    theta_b_dot = imu.data.gyroZ * PI / 180.0f;
    theta_b_accel = atan2(imu.data.accelX, imu.data.accelY);
    ///

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

    //add calc for theta_b 3D

   /*  Serial.print(theta_b, 6);
    Serial.print(",");
    Serial.println(theta_b_dot, 6); */
    delay(1);

}