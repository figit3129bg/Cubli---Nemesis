#include <Arduino.h>
#include "imu.h"
#include "edge.h"
#include "corner.h"
//#include "kalman.h"
//#include "servo/servo.h"
// jump.cpp has no functions yet -- nothing to call until it does.

#include <MoteusTeensy.h>

// Closed-loop balance controller for the 2D cubli, running entirely on the
// Teensy: reads theta_b/theta_b_dot from the onboard IMU, folds in the
// reaction wheel's own rate as reported by the moteus controller, and drives
// the wheel with an LQR state-feedback law over CAN-FD (see edge.cpp for the
// gains and control law).
//
// NOTE ON LOGGING: the Teensy has no filesystem to write cubli_log.csv to.
// Instead, this sketch prints one CSV row per loop over USB serial; a
// PC-side script (log_serial.py, in this same folder) reads that stream and
// writes cubli_log.csv to disk. See log_serial.py for the receiving half of
// this.

// CAN-FD bus: Teensy 4.1 CAN3 (pins 30/31), 1 Mbps arbitration, matching
// motors_test.cpp's wiring.
ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);


//moteus1--EDGE
Moteus controller1(canBus, []() {
    Moteus::Options options;
    options.id = 1;
    return options;
}());

//moteus2
//Moteus controller2(canBus, []() {
//    Moteus::Options options;
//    options.id = 2;
//    return options;
//}());

//moteus3
//Moteus controller3(canBus, []() {
//    Moteus::Options options;
//    options.id = 3;
//    return options;
//}());


// feedforward_torque/maximum_torque/kp_scale/kd_scale default to kIgnore
// (not sent); send them as floats, matching motors_test.cpp's format.
Moteus::PositionMode::Format commandFormat = []() {
    Moteus::PositionMode::Format format;
    format.feedforward_torque = Moteus::kFloat;
    format.maximum_torque     = Moteus::kFloat;
    format.kp_scale           = Moteus::kFloat;
    format.kd_scale           = Moteus::kFloat;
    return format;
}();





// Wall-clock reference for the time_s column, Teensy's equivalent of the PC
// side's std::chrono::steady_clock t_start. millis() wraps at ~49.7 days,
// which is fine for a balance-controller run.
unsigned long t_start_ms = 0;

// Safety ceiling sent to moteus with every command, regardless of which
// mode (edge/corner) computed the torque.
constexpr float kMaxTorqueNm = 1.00f;

void setup() {
  imu_setup(); 

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  controller1.SetStop();  // clear any faults before starting

  t_start_ms = millis();

  // Header row for log_serial.py on the PC side; must match the field order
  // printed at the bottom of loop().
  Serial.println("time_s,theta_b,theta_b_dot,theta_w_dot1,torque");
}

void loop() {
  // 1. read imu
  // 2. call jump/servo break -- brakeUpdate() below; brakeTrigger() not wired to a
  //                          condition yet, call it from wherever should fire the brake
  // 3. call edge control 
  // 4. break update
  // 5. jump to corner
  // 6. corner control 
  // 7. stop + break update 

  imu_loop();

  //float theta_hat, theta_b_dot_hat, theta_w_dot_hat;
  //KF(prev_torque, theta_b, theta_b_dot, theta_w_dot,
    // theta_hat, theta_b_dot_hat, theta_w_dot_hat);

  edge_loop();



  Moteus::PositionMode::Command command;
  command.position           = NAN;
  command.velocity           = NAN;
  command.kp_scale           = 0.0f;
  command.kd_scale           = 0.0f;
  command.feedforward_torque = torque_edge;
  command.maximum_torque     = kMaxTorqueNm;


  //change for controller1 SEND IT FROM EDGE FUNCTION
  const bool got_result = controller1.SetPosition(command, &commandFormat);
  if (got_result) {
    // moteus reports output revolutions/s; convert to rad/s to match
    // theta_b_dot's units for the next control cycle.
    theta_w_dot1 = controller1.last_result().values.velocity * 2.0f * PI;
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
  Serial.print(theta_w_dot1, 6);
  Serial.print(",");
  //Serial.println(torque_edge, 6);
  //Serial.print(",");
  Serial.println(torque_real_edge, 6);

////end EDGE for controller 1

//servo_brake


///start corner mode

//Moteus::PositionMode::Command command;
//command.position           = NAN;
//command.velocity           = NAN;
//command.kp_scale           = 0.0f;
//command.kd_scale           = 0.0f;
//command.feedforward_torque = torque_corner;
//command.maximum_torque     = kMaxTorqueNm;

//const bool got_result = controller2.SetPosition(command, &commandFormat);
//const bool got_result = controller3.SetPosition(command, &commandFormat);
//if (got_result) {
    // moteus reports output revolutions/s; convert to rad/s to match
    // theta_b_dot's units for the next control cycle.
//  theta_w_dot2 = controller2.last_result().values.velocity * 2.0f * PI;
// theta_w_dot3 = controller3.last_result().values.velocity * 2.0f * PI;
//} else {
    // "#" prefix marks this as a status line, not a CSV data row, so the
    // PC-side logger (log_serial.py) can tell it apart from the data below.
//  Serial.println("# no response from moteus!");
//}

//const float t_s = (millis() - t_start_ms) / 1000.0f;

//Serial.print(t_s, 6);
//Serial.print(",");
//Serial.print(theta_b, 6);
//Serial.print(",");
  //Serial.print(theta_b_dot, 6);
  //Serial.print(",");
//Serial.print(theta_w_dot1, 6);
//Serial.print(",");
  //Serial.println(torque_edge, 6);
  //Serial.print(",");
//Serial.println(torque_real_edge, 6);

}