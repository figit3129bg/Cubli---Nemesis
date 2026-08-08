#ifndef IMU_H
#define IMU_H

extern float theta_b;      // complementary-filtered body angle (rad)
extern float theta_b_dot;  // gyro-derived body angular rate (rad/s)

void imu_setup();
void imu_loop();

#endif
