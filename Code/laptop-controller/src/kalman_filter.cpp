// kalman_filter.cpp
#include "kalman_filter.h"
#include <cmath>

CubliKalmanFilter::CubliKalmanFilter() {
    xhat_.setZero();
    P_ = Eigen::Vector3d(0.01, 0.01, 0.1).asDiagonal();

    const double mb = 0.199, mw = 0.065, l = 0.114551;
    const double Ib = 0.0009084, Iw = 0.0002035;
    const double Cb = 0.00102, Cw = 0.00005;
    const double g = 9.81, Km = 0.021;
    const double dt = 0.001; // must match actual loop rate - see note above

    const double denom = Ib + mw * l * l;

    Eigen::Matrix3d A;
    A << 0.0, 1.0, 0.0,
         (mb * l + mw * l) * g / denom, -Cb / denom, Cw / denom,
         -(mb * l + mw * l) * g / denom, Cb / denom, -(Ib + Iw + mw * l * l) * Cw / (Iw * denom);

    Eigen::Vector3d B;
    B << 0.0,
         -Km / denom,
         Km * (Ib + Iw + mw * l * l) / (Iw * denom);

    Ad_ = Eigen::Matrix3d::Identity() + A * dt;
    Bd_ = B * dt;
    H_  = Eigen::Matrix3d::Identity();

    Qkf_ = Eigen::Vector3d(1e-6, 1e-4, 1e-5).asDiagonal();

    const double deg2rad = M_PI / 180.0;
    Rkf_ = Eigen::Vector3d(std::pow(0.5 * deg2rad, 2), 5.8e-3, 3e-2).asDiagonal();
}

StateEstimate CubliKalmanFilter::update(const KFInput& in) {
    // Prediction
    Eigen::Vector3d x_pred = Ad_ * xhat_ + Bd_ * in.Tm;
    Eigen::Matrix3d P_pred = Ad_ * P_ * Ad_.transpose() + Qkf_;

    // Measurement
    Eigen::Vector3d z(in.theta_meas, in.theta_b_dot_meas, in.theta_w_dot_meas);

    // Kalman gain
    Eigen::Matrix3d S = H_ * P_pred * H_.transpose() + Rkf_;
    Eigen::Matrix3d L = P_pred * H_.transpose() * S.inverse();

    // Update (Joseph form, matches the MATLAB)
    xhat_ = x_pred + L * (z - H_ * x_pred);
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    P_ = (I3 - L * H_) * P_pred * (I3 - L * H_).transpose() + L * Rkf_ * L.transpose();
    P_ = 0.5 * (P_ + P_.transpose()); // force symmetry

    return { xhat_(0), xhat_(1), xhat_(2) };
}