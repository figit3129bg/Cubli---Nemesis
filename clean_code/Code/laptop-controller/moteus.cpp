#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include "moteus.h"

using namespace mjbots;

std::atomic<bool> g_running{true};

void SignalHandler(int) {
  g_running = false;
}

int main(int argc, char** argv) {
  moteus::Controller::DefaultArgProcess(argc, argv);

  std::signal(SIGINT, SignalHandler);  // Ctrl+C triggers clean shutdown

  moteus::Controller::Options options;
  options.id = 1;
  options.position_format.feedforward_torque = moteus::kFloat;
  options.position_format.maximum_torque     = moteus::kFloat;
  options.position_format.kp_scale           = moteus::kFloat;
  options.position_format.kd_scale           = moteus::kFloat;

  moteus::Controller controller(options);
  controller.SetStop();

  const double target_torque_Nm = 0.01;
  const double max_torque_Nm    = 0.01;

  // Command is built ONCE, outside the loop
  moteus::PositionMode::Command command;
  command.position            = std::numeric_limits<double>::quiet_NaN();
  command.velocity            =  20;     //std::numeric_limits<double>::quiet_NaN();
  command.kp_scale            = 0.0;
  command.kd_scale            = 1.0;
  command.feedforward_torque  = target_torque_Nm;
  command.maximum_torque      = max_torque_Nm;

  // Infinite loop just re-sends the SAME command struct as a heartbeat
  while (g_running) {
    const auto maybe_result = controller.SetPosition(command);

    if (maybe_result) {
      const auto& v = maybe_result->values;
      std::cout //<< "pos=" << v.position
                << " vel=" << v.velocity
                << " torque=" << v.torque
                //<< " mode=" << static_cast<int>(v.mode)
                //<< " fault=" << static_cast<int>(v.fault)
                
                << "\n";
    } else {
      std::cout << "no response!\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "Stopping...\n";
  controller.SetStop();
  return 0;
}

/* 






 #include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "moteus.h"

using namespace mjbots;

int main(int argc, char** argv) {
  moteus::Controller::DefaultArgProcess(argc, argv);

  moteus::Controller::Options options;
  options.id = 1;  // CAN-FD id of your moteus controller
  options.position_format.feedforward_torque = moteus::kFloat;
  options.position_format.maximum_torque     = moteus::kFloat;
  options.position_format.kp_scale           = moteus::kFloat;
  options.position_format.kd_scale           = moteus::kFloat;

  moteus::Controller controller(options);

  // Always start from a clean state
  controller.SetStop();

  const double target_torque_Nm = 0.5;   // your desired constant torque
  const double max_torque_Nm    = 1.0;   // safety ceiling, must be >= target_torque_Nm

  moteus::PositionMode::Command command;
  command.position          = std::numeric_limits<double>::quiet_NaN();  // no position control
  command.velocity          = std::numeric_limits<double>::quiet_NaN();  // no velocity control
  command.kp_scale          = 0.0;   // kill position feedback term
  command.kd_scale          = 0.0;   // kill velocity feedback term
  command.feedforward_torque = target_torque_Nm;
  command.maximum_torque    = max_torque_Nm;

 
  // moteus times out after servo.default_timeout_s (100ms default) without
  // a new command, so this loop must keep sending continuously
  for (int i = 0; i < 500; ++i) {
       const auto maybe_result = controller.SetPosition(command);



    if (maybe_result) {
      const auto& v = maybe_result->values;
      std::cout << "pos=" << v.position
                << " vel=" << v.velocity
                << " torque=" << v.torque
                << " mode=" << static_cast<int>(v.mode)
                << " fault=" << static_cast<int>(v.fault)
                << "\n";
    } else {
      std::cout << "no response!\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
  }

  controller.SetStop();
  return 0;
}  */