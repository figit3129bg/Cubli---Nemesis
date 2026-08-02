// imu_preprocessing.cpp
#include "imu_preprocessing.h"
#include <cmath>

KFInput imuSampleToKFInput(const ImuSample& sample, double Tm, double theta_w_dot_meas) {
    KFInput in;

    // PLACEHOLDER axis mapping: assumes rotation about IMU X-axis,
    // gravity splitting between Y and Z. Confirm by hand-rotating the
    // board about the intended pivot axis and checking theta_meas moves
    // while the other reading stays flat.
    in.theta_meas = std::atan2(sample.accelY, sample.accelZ);

    const double deg2rad = M_PI / 180.0;
    in.theta_b_dot_meas = sample.gyroX * deg2rad;

    in.Tm = Tm;
    in.theta_w_dot_meas = theta_w_dot_meas;

    return in;
}