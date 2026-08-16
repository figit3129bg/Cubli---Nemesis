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
    delay(1000);

    // const int N = 200;
    // double sum = 0;
    // for (int i = 0; i < N; i++)
    // {
    //     imu.getSensorData();
    //     sum += atan2(imu.data.accelX, imu.data.accelY);
    //     delay(5);
    // }
    // theta_offset = sum / N;
    theta_offset = -2.3681;

    Serial.print("theta_offset (rad): ");
    Serial.print(theta_offset, 4);
    Serial.print("  (deg): ");
    Serial.println(theta_offset * 180.0 / PI, 2);

    //delay(3000);

}

void imu_setup()
{
    Serial.begin(115200);
    // while (!Serial) { delay(10); }  // blocks forever with no monitor attached -- see chat

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
constexpr float kK1 = -13.0356;   // theta_b     [rad]   -> N*m
constexpr float kK2 = -2.3862;   // theta_b_dot [rad/s] -> N*m
constexpr float kK3 = -0.0003; 

// constexpr float kK1 = -10.0468;   // theta_b     [rad]   -> N*m
// constexpr float kK2 = -0.5838;   // theta_b_dot [rad/s] -> N*m
// constexpr float kK3 = -0.0003; 


// Safety ceiling sent to moteus with every command; also clamps the
// computed control effort so a bad IMU sample can't demand a torque spike.
constexpr float kMaxTorqueNm = 0.25f;

// Reaction-wheel speed ceiling [rad/s] -- tune to your wheel/motor's safe
// max. Torque that would accelerate the wheel further once it's past this
// is zeroed; torque that slows it back down is still allowed through.
constexpr float kMaxWheelSpeedRadPerSec = 200.0f;

float theta_w_dot = 0.0f;  // reaction wheel rate, fed back from moteus
float prev_torque = 0.0f;  // torque commanded last cycle; the KF's input Tm

// Safety brake: set the moment 's'/'S' is received over serial (see
// checkSafetyStop() below). Once latched, loop() keeps issuing SetStop()
// and skips the control code entirely -- power-cycle/reset the Teensy to
// resume normal operation.
bool motorsStopped = false;

// Poll serial for the 's'/'S' emergency-stop command. Called first thing
// every loop() so it's checked (and acted on) every cycle regardless of
// where the rest of the control loop is.
void checkSafetyStop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 's' || c == 'S') {
            motorsStopped = true;
            controller.SetStop();
            Serial.println("!!! SAFETY STOP: motors halted immediately !!!");
        }
    }
}

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
    //Serial.println("Send 's' or 'S' at any time to immediately stop the motors (safety brake).");
   // Serial.println("theta_b,theta_b_dot,theta_w_dot,torque,torque_raw,delta_t");
}

void loop() {
    checkSafetyStop();
    if (motorsStopped) {
        // Keep telling the moteus to stop every cycle instead of trusting a
        // single command to have landed.
        controller.SetStop();
        delay(10);
        return;
    }

    // Time to run one loop() iteration (measured loop-to-loop, since the
    // Arduino core does effectively nothing between calls): start of this
    // loop() minus start of the previous one, in seconds.
    static uint32_t t_loop_prev_us = micros();
    const uint32_t t_loop_now_us = micros();
    const float delta_t = (t_loop_now_us - t_loop_prev_us) * 1e-6f;
    t_loop_prev_us = t_loop_now_us;

    // The CSV header only prints once, in setup() -- reprint it periodically
    // so a logger (log_serial.py / pio device monitor) attaching after boot
    // still catches it instead of dropping every data row forever.
    static uint32_t t_header_prev_ms = 0;
    if (millis() - t_header_prev_ms >= 2000) {
        t_header_prev_ms = millis();
        //Serial.println("theta_b,theta_b_dot,theta_w_dot,torque,torque_raw,delta_t");
    }

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


    float torque_raw = kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot;


    float torque = constrain(
        kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot,
        -kMaxTorqueNm, kMaxTorqueNm);

    // Wheel speed ceiling: stop pushing it faster once it's already at the
    // limit, but still allow torque that slows it back down.
    if (theta_w_dot >= kMaxWheelSpeedRadPerSec && torque > 0.0f) {
        torque = 0.0f;
        //Serial.print("SPEED TOO HIGH! SPEED TOO HIGH! SPEED TOO HIGH!");

    } else if (theta_w_dot <= -kMaxWheelSpeedRadPerSec && torque < 0.0f) {
        torque = 0.0f;
        //Serial.print("SPEED TOO -HIGH! SPEED TOO -HIGH! SPEED TOO -HIGH!");
    }
    prev_torque = torque;

    Moteus::PositionMode::Command command;
    command.position           = NAN;
    command.velocity           = NAN;
    command.kp_scale           = 0.0f;
    command.kd_scale           = 0.0f;
    command.feedforward_torque = 0.02f;
    command.maximum_torque     = kMaxTorqueNm;

    // Moteus::QueryCommand qc;
    // qc.q_current = Moteus::kFloat;
    // qc.d_current = Moteus::kFloat;

    //float qc_current1 = qc.q_current;

    float reported_torque = 0.0f;
    const bool got_result = controller.SetPosition(command, &commandFormat);
    if (got_result) {
        // moteus reports output revolutions/s; convert to rad/s to match
        // theta_b_dot's units for the next control cycle.
        theta_w_dot = controller.last_result().values.velocity * 2.0f * PI;
        reported_torque = controller.last_result().values.torque;
    } else {
        Serial.println("no response from moteus!");
    }

    // Serial.print(theta_w_dot, 6);
    // Serial.print(",");
    // Serial.print(torque_raw, 6);
    // Serial.print(",");
    Serial.print(qc_current1, 6);
    Serial.print(",");
    Serial.print(theta_w_dot, 6);
    Serial.print(",");
    Serial.print(reported_torque - torque_raw, 6);
    Serial.print("\n");
    
    
    //  Serial.print(theta_b, 6);
    //  Serial.print(",");
    //  Serial.print(theta_b_dot, 6);
    //  Serial.print(",");
    //  Serial.print(theta_w_dot, 6);
    //  Serial.print(",");
    //  Serial.print(torque, 6);
    //  Serial.print(",");
    //  Serial.print(torque_raw, 6);
    //  Serial.print(",");
    //  Serial.println(delta_t, 6);
}