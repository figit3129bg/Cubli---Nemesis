// types.h
#pragma once

struct ImuSample {
    double accelX, accelY, accelZ; // g's
    double gyroX, gyroY, gyroZ;    // deg/s
};

struct KFInput {
    double Tm;               // motor torque, N*m (constant/0 for now)
    double theta_meas;        // rad
    double theta_b_dot_meas;  // rad/s
    double theta_w_dot_meas;  // rad/s (constant/0 for now - no wheel encoder yet)
};

struct StateEstimate {
    double theta;
    double theta_b_dot;
    double theta_w_dot;
};