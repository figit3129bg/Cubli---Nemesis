// test_kf_standalone.cpp
#include <iostream>
#include "kalman_filter.h"

int main() {
    CubliKalmanFilter kf;

    // All inputs are constants for now - no serial link, no motor, no encoder.
    KFInput in;
    in.Tm = 0.0;
    in.theta_meas = 0.05;        // pretend the cube is tilted ~2.9 deg
    in.theta_b_dot_meas = 0.0;
    in.theta_w_dot_meas = 0.0;

    for (int i = 0; i < 200; ++i) {
        StateEstimate est = kf.update(in);
        std::cout << "theta=" << est.theta
                  << " theta_b_dot=" << est.theta_b_dot
                  << " theta_w_dot=" << est.theta_w_dot << "\n";
    }
    return 0;
}