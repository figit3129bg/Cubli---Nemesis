// Kalman filter
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// Runs one step of the 3-state Kalman filter (theta_b, theta_b_dot, theta_w_dot).
// Tm                : motor torque command used in this step (the KF's control input)
// theta_b_meas      : measured body angle
// theta_b_dot_meas  : measured body angular rate
// theta_w_dot_meas  : measured wheel angular rate
// theta_b_hat, theta_b_dot_hat, theta_w_dot_hat : outputs (passed by reference)
void kalmanFilter(double Tm,
                   double theta_b_meas,
                   double theta_b_dot_meas,
                   double theta_w_dot_meas,
                   double &theta_b_hat,
                   double &theta_b_dot_hat,
                   double &theta_w_dot_hat);

// Optional: call this if you ever need to reset the filter's internal state
// (equivalent to clearing the "persistent" variables in the MATLAB version).
void kalmanFilterReset();

#endif // FUNCTIONS_H