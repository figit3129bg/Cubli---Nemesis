#include <Arduino.h>
#include "imu.h"
#include "kalman.h"

#include <MoteusTeensy.h>

// Closed-loop balance controller for the 2D cubli, running entirely on the
// Teensy: reads theta_b/theta_b_dot from the onboard IMU, folds in the
// reaction wheel's own rate as reported by the moteus controller, and drives
// the wheel with an LQR state-feedback law over CAN-FD (see
// ../host/lqr/lqr.cpp for the original PC-side version and gain notes).

// CAN-FD bus: Teensy 4.1 CAN3 (pins 30/31), 1 Mbps arbitration, matching
// motor_test.cpp's wiring.
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
constexpr float kK1 = -270.5849f;  // theta_b     [rad]   -> N*m
constexpr float kK2 = -21.5554f;   // theta_b_dot [rad/s] -> N*m
constexpr float kK3 = -1.0024f;    // theta_w_dot [rad/s] -> N*m

// Safety ceiling sent to moteus with every command; also clamps the
// computed control effort so a bad IMU sample can't demand a torque spike.
constexpr float kMaxTorqueNm = 5.0f;

float theta_w_dot = 0.0f;  // reaction wheel rate, fed back from moteus
float prev_torque = 0.0f;  // torque commanded last cycle; the KF's input Tm

void setup() {
  imu_setup();

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  controller.SetStop();  // clear any faults before starting
}

void loop() {
  imu_loop();

  float theta_hat, theta_b_dot_hat, theta_w_dot_hat;
  KF(prev_torque, theta_b, theta_b_dot, theta_w_dot,
     theta_hat, theta_b_dot_hat, theta_w_dot_hat);

  const float torque = constrain(
      kK1 * theta_hat + kK2 * theta_b_dot_hat + kK3 * theta_w_dot_hat,
      -kMaxTorqueNm, kMaxTorqueNm);
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

  Serial.print(theta_hat, 6);
  Serial.print(",");
  Serial.print(theta_b_dot_hat, 6);
  Serial.print(",");
  Serial.print(theta_w_dot_hat, 6);
  Serial.print(",");
  Serial.println(torque, 6);
}
