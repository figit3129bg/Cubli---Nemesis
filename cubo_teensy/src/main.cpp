#include <Arduino.h>
#include "imu.h"
//#include "kalman.h"
//#include "servo/servo.h"
// jump.cpp has no functions yet -- nothing to call until it does.

#include <MoteusTeensy.h>

// Closed-loop balance controller for the 2D cubli, running entirely on the
// Teensy: reads theta_b/theta_b_dot from the onboard IMU, folds in the
// reaction wheel's own rate as reported by the moteus controller, and drives
// the wheel with an LQR state-feedback law over CAN-FD (see
// ../host/lqr/lqr.cpp for the original PC-side version and gain notes).
//
// NOTE ON LOGGING: the Teensy has no filesystem to write cubli_log.csv to
// (unlike the PC-side program in host/lqr/main.cpp, which can open a real
// ofstream). Instead, this sketch prints one CSV row per loop over USB
// serial; a PC-side script (log_serial.py, in this same folder) reads that
// stream and writes cubli_log.csv to disk. See log_serial.py for the
// receiving half of this.

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
constexpr float kK1 = 1.7481f;   // theta_b     [rad]   -> N*m
constexpr float kK2 = 0.3349f;   // theta_b_dot [rad/s] -> N*m
constexpr float kK3 = 0.001f; 




// Safety ceiling sent to moteus with every command; also clamps the
// computed control effort so a bad IMU sample can't demand a torque spike.
constexpr float kMaxTorqueNm = 1.00f;

// Reaction-wheel speed ceiling [rad/s] -- tune to your wheel/motor's safe
// max. Torque that would accelerate the wheel further once it's past this
// is zeroed; torque that slows it back down is still allowed through.
constexpr float kMaxWheelSpeedRadPerSec = 150.0f;

float theta_w_dot = 0.0f;  // reaction wheel rate, fed back from moteus
float prev_torque = 0.0f;  // torque commanded last cycle; the KF's input Tm

// Wall-clock reference for the time_s column, Teensy's equivalent of the PC
// side's std::chrono::steady_clock t_start. millis() wraps at ~49.7 days,
// which is fine for a balance-controller run.
unsigned long t_start_ms = 0;

void setup() {
  imu_setup(); 

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  controller.SetStop();  // clear any faults before starting

  t_start_ms = millis();

  // Header row for log_serial.py on the PC side; must match the field order
  // printed at the bottom of loop().
  Serial.println("time_s,theta_b,theta_b_dot,theta_w_dot,torque");
}

void loop() {

  // 1. call jump          -- TODO: jump.cpp has no code yet
  // 2. call servo brake   -- brakeUpdate() below; brakeTrigger() not wired to a
  //                          condition yet, call it from wherever should fire the brake
  // 3. call stabilization code (lqr)
  // end loop
  //brakeUpdate();
  imu_loop();

  //float theta_hat, theta_b_dot_hat, theta_w_dot_hat;
  //KF(prev_torque, theta_b, theta_b_dot, theta_w_dot,
    // theta_hat, theta_b_dot_hat, theta_w_dot_hat);

  float torque = constrain(
      kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot,
      -kMaxTorqueNm, kMaxTorqueNm);

  float torque_real = kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot;
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
    // "#" prefix marks this as a status line, not a CSV data row, so the
    // PC-side logger (log_serial.py) can tell it apart from the data below.
    Serial.println("# no response from moteus!");
  }

  const float t_s = (millis() - t_start_ms) / 1000.0f;

  Serial.print(t_s, 6);
  Serial.print(",");
  Serial.print(theta_b, 6);
  Serial.print(",");
  //Serial.print(theta_b_dot, 6);
  //Serial.print(",");
  Serial.print(theta_w_dot, 6);
  Serial.print(",");
  //Serial.println(torque, 6);
  //Serial.print(","); 
  Serial.println(torque_real, 6);
}