// IMU reading:

#include <Wire.h>
#include <Arduino.h>
#include <MoteusTeensy.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <cmath>

BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;

float theta_b_dot, theta_b_accel, theta_b;
float theta_offset = 0.0f;

float wrapToPi(float angle)
{
    while (angle > PI)  angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

void CalibrateZero() {
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

    CalibrateZero();
}

void imu_loop() {
    static float theta_b_prev = 0.0f;
    static uint32_t t_prev_us = 0;
    static bool first_run = true;

    imu.getSensorData();

    uint32_t t_now_us = micros();
    float dt = (t_now_us - t_prev_us) * 1e-6f;
    t_prev_us = t_now_us;

    theta_b_dot = imu.data.gyroZ * PI / 180.0f;
    theta_b_accel = atan2(imu.data.accelX, imu.data.accelY);

    float z = wrapToPi(theta_b_accel - theta_offset);

    if (first_run)
    {
        theta_b_prev = z;
        theta_b = z;
        first_run = false;
        return;
    }

    float theta_pred = wrapToPi(theta_b_prev + theta_b_dot * dt);
    theta_b = wrapToPi(theta_pred + 0.04f * wrapToPi(z - theta_pred));
    theta_b_prev = theta_b;
}

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

Moteus controller(canBus, []() {
    Moteus::Options options;
    options.id = 1;
    return options;
}());

// feedforward_torque/maximum_torque/kp_scale/kd_scale default to kIgnore
// (not sent); send them as floats, matching motor_test.cpp's format.
Moteus::PositionMode::Format commandFormat = []() {
    Moteus::PositionMode::Format format;
    format.feedforward_torque = Moteus::kFloat;
    format.maximum_torque     = Moteus::kFloat;
    format.kp_scale           = Moteus::kFloat;
    format.kd_scale           = Moteus::kFloat;
    return format;
}();

// LQR state-feedback gains (theta_b, theta_b_dot, theta_w_dot -> torque).
constexpr float kK1 = 1.7481f;   // theta_b     [rad]   -> N*m
constexpr float kK2 = 0.3349f;   // theta_b_dot [rad/s] -> N*m
constexpr float kK3 = 0.001f; 

// Safety ceiling sent to moteus with every command; also clamps the
// computed control effort so a bad IMU sample can't demand a torque spike.
constexpr float kMaxTorqueNm = 5.0f;

// Reaction-wheel speed ceiling [rad/s] -- tune to your wheel/motor's safe
// max. Torque that would accelerate the wheel further once it's past this
// is zeroed; torque that slows it back down is still allowed through.
constexpr float kMaxWheelSpeedRadPerSec = 40.0f;

float theta_w_dot = 0.0f;  // reaction wheel rate, fed back from moteus
float prev_torque = 0.0f;  // torque commanded last cycle; the KF's input Tm

void setup() {
    imu_setup();
    // servo_setup();

    const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
    while (errorCode != 0) {
        Serial.print("CAN error 0x");
        Serial.println(errorCode, HEX);
        delay(1000);
    }

    controller.SetStop();  // clear any faults before starting
    Serial.println("theta_b,theta_b_dot,theta_w_dot,torque");
}

void loop() {
    // 1. call jump          -- TODO: jump.cpp has no code yet
    // 2. call servo brake   -- brakeUpdate() below; brakeTrigger() not wired
    //                          to a condition yet, call it from wherever
    //                          should fire the brake
    // 3. call stabilization code (lqr)
    // end loop
    // brakeUpdate();

    imu_loop();

    // float theta_hat, theta_b_dot_hat, theta_w_dot_hat;
    // KF(prev_torque, theta_b, theta_b_dot, theta_w_dot,
    //    theta_hat, theta_b_dot_hat, theta_w_dot_hat);

    float torque = constrain(
        kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot,
        -kMaxTorqueNm, kMaxTorqueNm);

    // Wheel speed ceiling: stop pushing it faster once it's already at the
    // limit, but still allow torque that slows it back down.
    if (theta_w_dot >= kMaxWheelSpeedRadPerSec && torque > 0.0f) {
        torque = 0.0f;
    } else if (theta_w_dot <= -kMaxWheelSpeedRadPerSec && torque < 0.0f) {
        torque = 0.0f;
    }
    prev_torque = torque;

    Moteus::PositionMode::Command command;
    command.position           = NAN;
    command.velocity           = NAN;
    command.kp_scale           = 0.0f;
    command.kd_scale           = 0.0f;
    command.feedforward_torque = torque;
    command.maximum_torque     = kMaxTorqueNm;

    const bool got_result = controller.SetPosition(command, &commandFormat);
    if (got_result) {
        // moteus reports output revolutions/s; convert to rad/s to match
        // theta_b_dot's units for the next control cycle.
        theta_w_dot = controller.last_result().values.velocity * 2.0f * PI;
    } else {
        Serial.println("no response from moteus!");
    }

    Serial.print(theta_b, 6);
    Serial.print(",");
    Serial.print(theta_b_dot, 6);
    Serial.print(",");
    Serial.print(theta_w_dot, 6);
    Serial.print(",");
    Serial.println(torque, 6);
}