#include <iostream>
#include <unistd.h>
#include <limits>
#include <csignal>
#include <atomic>
#include <cmath>
#include "moteus.h"

namespace moteus = mjbots::moteus;

std::atomic<bool> g_running{true};

void signal_handler(int) {
  g_running = false;
}

int main(int argc, char** argv) {
  moteus::Controller::DefaultArgProcess(argc, argv);
  std::signal(SIGINT, signal_handler);

  moteus::Controller c([]() {
    moteus::Controller::Options options;
    options.id = 1;
    return options;
  }());

  moteus::PositionMode::Command command;
  command.position = std::numeric_limits<double>::quiet_NaN();
  command.velocity = 40.0; // spinning in reverse

  double current_velocity_cmd = command.velocity;

  while (g_running) {
    command.velocity = current_velocity_cmd;
    const auto maybe_result = c.SetPosition(command);
    if (maybe_result) {
      const auto& v = maybe_result->values;
      std::cout << "Mode: "  << static_cast<int>(v.mode)
                << " Fault: " << static_cast<int>(v.fault)
                << " Position: " << v.position
                << " Velocity: " << v.velocity
                << "\n";
    } else {
      std::cout << "No response\n";
    }
    ::usleep(20000); // ~50Hz
  }

  // Ramp velocity down to zero under servo control before cutting torque
  std::cout << "Ramping down to stop...\n";
  const double ramp_step = 0.1  ;      // rev/s per 20ms tick (~2.5 rev/s^2 decel)
  while (std::abs(current_velocity_cmd) > 1e-3) {
    if (current_velocity_cmd > 0) {
      current_velocity_cmd = std::max(0.0, current_velocity_cmd - ramp_step);
    } else {
      current_velocity_cmd = std::min(0.0, current_velocity_cmd + ramp_step);
    }
    command.velocity = current_velocity_cmd;
    c.SetPosition(command);
    ::usleep(20000);
  }

  // Now that velocity is ~0, cut torque and stop
  std::cout << "Stopping motor...\n";
  c.SetStop();

  return 0;
}