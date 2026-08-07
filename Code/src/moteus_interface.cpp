#include "moteus_interface.h"
#include "MoteusUart.h"
#include "Moteus.h"

using MoteusCtrl = MoteusController<MoteusUart<HardwareSerial>>;

double theta_w_dot = 0.0;

namespace {
MoteusUart<HardwareSerial> uart_bus(Serial2);

MoteusCtrl controller(uart_bus, []() {
  MoteusCtrl::Options options;
  options.id = 1;
  // UART connections can lose responses, so retry diagnostic reads.
  options.diagnostic_retry_count = 3;
  return options;
}());

constexpr double kMaxTorqueNm = 0.40;
}  // namespace

void moteusSetup() {
  uart_bus.begin();
  controller.SetStop();
}

void moteusSendTorque(double torque) {
  MoteusCtrl::PositionMode::Command cmd;
  cmd.position = std::numeric_limits<double>::quiet_NaN();
  cmd.velocity = 0.0;
  cmd.kp_scale = 0.0;
  cmd.kd_scale = 0.0;
  cmd.ilimit_scale = 0.0;
  cmd.feedforward_torque = torque;
  cmd.maximum_torque = kMaxTorqueNm;
  controller.SetPosition(cmd);
}
