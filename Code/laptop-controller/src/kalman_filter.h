// kalman_filter.h
#pragma once
#include <Eigen/Dense>
#include "types.h"

class CubliKalmanFilter {
public:
    CubliKalmanFilter();
    StateEstimate update(const KFInput& in);

private:
    Eigen::Vector3d xhat_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Ad_;
    Eigen::Vector3d Bd_;
    Eigen::Matrix3d H_;
    Eigen::Matrix3d Qkf_;
    Eigen::Matrix3d Rkf_;
};