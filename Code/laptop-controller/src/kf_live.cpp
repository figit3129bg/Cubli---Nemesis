// kf_live.cpp
// Reads live IMU CSV lines from esp32-imu-mode (accelX,accelY,accelZ,gyroX,gyroY,gyroZ)
// over serial and feeds them through the Kalman filter in real time.
#include <iostream>
#include <sstream>
#include <string>
#include "imu_preprocessing.h"
#include "kalman_filter.h"
#include "serial_port.h"

namespace {

// Parses one CSV line as printed by esp32-imu-mode/src/imu_bmi270.cpp.
// Returns false on a malformed line (e.g. a partial line right after connecting).
bool parseImuLine(const std::string& line, ImuSample& out) {
    std::stringstream ss(line);
    std::string field;
    double values[6];
    for (double& v : values) {
        if (!std::getline(ss, field, ',')) return false;
        try {
            v = std::stod(field);
        } catch (const std::exception&) {
            return false;
        }
    }
    out.accelX = values[0]; out.accelY = values[1]; out.accelZ = values[2];
    out.gyroX  = values[3]; out.gyroY  = values[4]; out.gyroZ  = values[5];
    return true;
}

} // namespace

int main(int argc, char** argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const int baud = 115200; // must match Serial.begin() in imu_bmi270.cpp

    SerialPort port(device, baud);
    std::cerr << "Connected to " << device << " at " << baud << " baud.\n";

    CubliKalmanFilter kf;

    while (true) {
        std::string line = port.readLine();

        ImuSample sample;
        if (!parseImuLine(line, sample)) continue;

        // No motor torque command or wheel encoder yet - both held at 0.
        KFInput in = imuSampleToKFInput(sample, /*Tm=*/0.0, /*theta_w_dot_meas=*/0.0);
        StateEstimate est = kf.update(in);

        std::cout << "theta=" << est.theta
                  << " theta_b_dot=" << est.theta_b_dot
                  << " theta_w_dot=" << est.theta_w_dot << "\n";
    }
}
