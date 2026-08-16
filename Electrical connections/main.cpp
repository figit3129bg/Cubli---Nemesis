#include <iostream>
#include <limits>
#include <thread>     // portable sleep, works on Windows/Mac/Linux
#include <chrono>

#include "moteus.h"

// Portable microsecond sleep (replaces POSIX-only usleep, which doesn't
// exist on Windows/MSVC).
void SleepMicroseconds(int us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

namespace moteus = mjbots::moteus;

int main(int argc, char** argv) {
  // Lets the library pick up standard command-line flags, e.g.
  // --transport (auto-detects an fdcanusb, pi3hat, or socketcan device).
  moteus::Controller::DefaultArgProcess(argc, argv);

  // Create a controller object for the moteus with CAN id 1.
  // (Change options.id if your moteus is configured with a different id.)
  moteus::Controller controller([]() {
    moteus::Controller::Options options;
    options.id = 1;
    return options;
  }());

  // Always clear any fault and put the controller in a known "stopped"
  // state before commanding motion. Good practice on every startup.
  controller.SetStop();
  SleepMicroseconds(100000);  // give it a moment to process

  // --- "Zero" the position reference ---
  // On power-up, moteus treats wherever the rotor currently is as an
  // arbitrary starting position (not necessarily 0). SetOutputExact/SetRezero
  // tells the controller "call the current physical position 0.0" so that
  // all subsequent position commands are relative to this Instant.
  controller.SetOutputExact(0.0);
  SleepMicroseconds(100000);

  // --- Build a position-mode command ---
  moteus::PositionMode::Command command;

  // position = 0.0 means "hold/move to position 0.0 revolutions" (relative
  // to the zero point we just set). Using NaN instead of a number here
  // would mean "don't actively control position, just obey velocity/torque".
  command.position = 0.0;

  // velocity is the feed-forward velocity added to the position controller;
  // 0.0 here just means "no extra velocity term, use position control only".
  command.velocity = 0.0;

  // Optional: cap how fast/hard it will move to get there.
  // command.maximum_torque = 2.0;   // Nm, uncomment & tune for your motor
  // command.velocity_limit = 2.0;   // rev/s

  std::cout << "Commanding position 0.0 ...\n";

  // Send the command repeatedly. moteus expects a steady stream of CAN
  // frames (it has a watchdog timeout and will stop the motor if commands
  // stop arriving), so this is normally done in a loop running at
  // 100Hz-1kHz depending on your application.
  for (int i = 0; i < 200; ++i) {
    const auto maybe_result = controller.SetPosition(command);

    if (maybe_result) {
      const auto& v = maybe_result->values;
      std::cout << "mode=" << static_cast<int>(v.mode)
                << " fault=" << static_cast<int>(v.fault)
                << " position=" << v.position
                << " velocity=" << v.velocity
                << "\n";
    } else {
      std::cout << "No response from controller!\n";
    }

    SleepMicroseconds(10000);  // ~100Hz command rate
  }

  // Stop the motor cleanly when done.
  controller.SetStop();

  return 0;
}