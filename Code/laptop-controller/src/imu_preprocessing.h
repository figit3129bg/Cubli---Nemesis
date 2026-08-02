// imu_preprocessing.h
#pragma once
#include "types.h"

// Converts a raw IMU sample into the theta/theta_dot measurements the KF expects.
// AXIS MAPPING IS A PLACEHOLDER - verify once the BMI270 is mounted in the frame.
KFInput imuSampleToKFInput(const ImuSample& sample, double Tm, double theta_w_dot_meas);