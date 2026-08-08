// Closed-loop balance controller for the 2D cubli.
//
// Reads theta_b / theta_b_dot streamed by the ESP32 (Code/esp32-imu) over
// USB serial, folds in the reaction wheel's own rate as reported by the
// moteus controller, and drives the wheel with an LQR state-feedback law
// over CAN-FD via an fdcanusb dongle (see ../../info.txt for the wiring/
// setup notes).
//
// State vector: x = [theta_b, theta_b_dot, theta_w_dot]
//   theta_b     - body tilt away from the balance point, rad. The ESP32
//                 already zero-references this during its calibrateZero()
//                 step, so no additional setpoint offset is applied here.
//   theta_b_dot - body angular rate, rad/s.
//   theta_w_dot - reaction wheel angular rate, rad/s (fed back from moteus).
//
// Control law: torque = K1*theta_b + K2*theta_b_dot + K3*theta_w_dot
// Gains below were computed offline from the linearized cubli model
// (see the pre-cleanup Code/src/main.cpp derivation this was ported from).

#include "functions.h"
#include "serial_port.h"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <csignal>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <fstream>
#include <chrono>

#include "moteus.h"

using namespace mjbots;

namespace {

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

constexpr double kPi = 3.14159265358979323846;

// LQR state-feedback gains.
constexpr double kK1 = -199.2601;  // theta_b     [rad]   -> N*m
constexpr double kK2 = -17.4678;   // theta_b_dot [rad/s] -> N*m
constexpr double kK3 = -0.6348;    // theta_w_dot [rad/s] -> N*m

// Safety ceiling sent to moteus with every command; also clamps the
// computed control effort so a bad IMU sample can't demand a torque spike.
// Derived from this motor's own limits in current_config.cfg rather than
// guessed: Kt = 60 / (2*pi*Kv) = 60 / (2*pi*379.38) ~= 0.02517 Nm/A, and
// servo.max_current_A = 3 A, so max deliverable torque ~= 0.0755 Nm.
// Re-derive this if current_config.cfg's motor.Kv or servo.max_current_A
// ever change.
constexpr double kMaxTorqueNm = 2;

double torqueFromState(double theta_b, double theta_b_dot, double theta_w_dot) {
    return (kK1 * theta_b + kK2 * theta_b_dot + kK3 * theta_w_dot);
}

}  // namespace

int main(int argc, char** argv) {
    moteus::Controller::DefaultArgProcess(argc, argv);
    std::signal(SIGINT, SignalHandler);  // Ctrl+C triggers clean shutdown

    SerialPort port;
    if (!port.open("/dev/ttyUSB0", 115200)) {
        std::cerr << "Failed to open /dev/ttyUSB0 (ESP32 IMU link)" << std::endl;
        return 1;
    }

    // Per-run data log for data.py: raw measurements, the (currently
    // pass-through) KF outputs actually fed to the controller, and the
    // resulting torque command.
    std::ofstream csv("cubli_log.csv");
    if (!csv) {
        std::cerr << "Failed to open cubli_log.csv for logging" << std::endl;
        return 1;
    }
    csv << "time_s,theta_b,theta_b_dot,theta_w_dot,"
           "theta_b_hat,theta_b_dot_hat,theta_w_dot_hat,torque\n";
    const auto t_start = std::chrono::steady_clock::now();

    moteus::Controller::Options options;
    options.id = 1;
    options.position_format.feedforward_torque = moteus::kFloat;
    options.position_format.maximum_torque     = moteus::kFloat;
    options.position_format.kp_scale           = moteus::kFloat;
    options.position_format.kd_scale           = moteus::kFloat;

    moteus::Controller controller(options);
    controller.SetStop();

    double theta_b = 0.0;
    double theta_b_dot = 0.0;
    double theta_w_dot = 0.0;  // updated from the moteus query response below
    double prev_torque = 0.0;  // torque commanded last cycle; the KF's input Tm

    std::string line;
    while (g_running && port.readLine(line)) {
        // Expecting: "theta_b,theta_b_dot"  e.g.  "0.7854,0.0123"
        std::istringstream iss(line);
        std::string a, b;
        if (!(std::getline(iss, a, ',') && std::getline(iss, b, ','))) {
            continue;  // malformed/partial line (e.g. right after connecting)
        }

        try {
            theta_b = std::stod(a);
            theta_b_dot = std::stod(b);
        } catch (const std::exception&) {
            continue;
        }

        double theta_b_hat = 0.0;
        double theta_b_dot_hat = 0.0;
        double theta_w_dot_hat = 0.0;
        kalmanFilter(prev_torque, theta_b, theta_b_dot, theta_w_dot,
                     theta_b_hat, theta_b_dot_hat, theta_w_dot_hat);

        const double torque = std::clamp(
            torqueFromState(theta_b_hat, theta_b_dot_hat, theta_w_dot_hat),
            -kMaxTorqueNm, kMaxTorqueNm);
        prev_torque = torque;

        moteus::PositionMode::Command command;
        command.position           = std::numeric_limits<double>::quiet_NaN();
        command.velocity           = std::numeric_limits<double>::quiet_NaN();
        command.kp_scale           = 0.0;
        command.kd_scale           = 0.0;
        command.feedforward_torque = torque;
        command.maximum_torque     = kMaxTorqueNm;

        const double t_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        csv << t_s << ','
            << theta_b << ',' << theta_b_dot << ',' << theta_w_dot << ','
            << theta_b_hat << ',' << theta_b_dot_hat << ',' << theta_w_dot_hat << ','
            << torque << '\n';

        const auto maybe_result = controller.SetPosition(command);
        if (maybe_result) {
            // moteus reports velocity in output revolutions/s; convert to
            // rad/s to match theta_b_dot's units for the next control cycle.
            theta_w_dot = maybe_result->values.velocity * 2.0 * kPi;
        } else {
            std::cerr << "no response from moteus!" << std::endl;
        }

        std::cout << " theta_w_dot=" << theta_w_dot
                  << " torque=" << torque << std::endl;
    }

    csv.close();
    std::cout << "Stopping..." << std::endl;
    controller.SetStop();
    port.close();
    return 0;
}