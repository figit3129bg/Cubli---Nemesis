#include <Arduino.h>
#include "edge.h"
#include "imu.h"


constexpr float kK1 = 1.7481f;   // theta_b     [rad]   -> N*m
constexpr float kK2 = 0.3349f;   // theta_b_dot [rad/s] -> N*m
constexpr float kK3 = 0.001f;    // theta_w_dot1 [rad/s] -> N*m

// Safety ceiling sent to moteus with every command; also clamps the
// computed control effort so a bad IMU sample can't demand a torque spike.
constexpr float kMaxTorqueNm = 1.00f;

// Reaction-wheel speed ceiling [rad/s] -- tune to your wheel/motor's safe
// max. Torque that would accelerate the wheel further once it's past this
// is zeroed; torque that slows it back down is still allowed through.
constexpr float kMaxWheelSpeedRadPerSec = 150.0f;

float theta_w_dot1 = 0.0f;  // reaction wheel rate, fed back from moteus
float prev_torque_edge = 0.0f;  // torque commanded last cycle; the KF's input Tm
float torque_edge = 0.0f;       // clamped control effort, sent to controller1
float torque_real_edge = 0.0f;  // unclamped control effort, for logging

void edge_loop() {
 torque_edge = constrain(
      kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot1,
      -kMaxTorqueNm, kMaxTorqueNm);

  torque_real_edge = kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot1;
  if (theta_w_dot1 >= kMaxWheelSpeedRadPerSec && torque_edge > 0.0f) {
    torque_edge = 0.0f;
  } else if (theta_w_dot1 <= -kMaxWheelSpeedRadPerSec && torque_edge < 0.0f) {
    torque_edge = 0.0f;
  }
  prev_torque_edge = torque_edge;
}