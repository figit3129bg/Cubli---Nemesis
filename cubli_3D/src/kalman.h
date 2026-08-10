#ifndef KALMAN_H
#define KALMAN_H

// Reaction-wheel balance Kalman filter, ported from the MATLAB KF() function.
// State: [theta; theta_b_dot; theta_w_dot] (body angle, body rate, wheel rate).
void KF(float Tm, float theta_meas, float theta_b_dot_meas, float theta_w_dot_meas,
        float &theta_hat, float &theta_b_dot_hat, float &theta_w_dot_hat);

#endif
