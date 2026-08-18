#include "corner.h"
#include <math.h>

// ---------------------------------------------------------------------
// ASSUMED IMU INTERFACE -- adjust these two lines to match your imu.h.
// Expecting a unit attitude quaternion and body-frame gyro rates,
// refreshed each imu_loop() call.
// ---------------------------------------------------------------------
extern float q_w, q_x, q_y, q_z;       // attitude quaternion
extern float gyro_x, gyro_y, gyro_z;   // body angular rates [rad/s]

// ---------------------------------------------------------------------
// Reference ("corner-up") attitude. Identity assumes the IMU is zeroed
// with the cube balanced on the target corner at power-up. If you
// calibrate this at boot instead, set it in corner_setup() below.
// ---------------------------------------------------------------------
static float qd_w = 1.0f, qd_x = 0.0f, qd_y = 0.0f, qd_z = 0.0f;

// ---------------------------------------------------------------------
// LQR gain matrix, 3 (wheel torques) x 9 (states):
//   x = [ex, ey, ez, wx, wy, wz, wd1, wd2, wd3]
//     ex,ey,ez : small-angle attitude error (2 * vector part of q_err)
//     wx,wy,wz : body angular rate [rad/s]
//     wd1..3   : wheel angular rate [rad/s]
//   u = -K * x, u[i] = feedforward torque for wheel i [N*m]
//
// PLACEHOLDER -- paste the K from your MATLAB/Python `lqr(A, B, Q, R)`
// design here. Row i is wheel i's gains; assumes orthogonal wheel
// mounting aligned with body axes (see note at bottom if not).
// ---------------------------------------------------------------------
static const float K[3][9] = {
    // ex     ey     ez     wx     wy     wz     wd1    wd2    wd3
    {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f },
    {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f },
    {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f },
};

// Software ceiling independent of each command's own maximum_torque --
// keeps a bad state/gain glitch from commanding something huge before it
// even reaches the CAN bus.
constexpr float kMaxCornerTorqueNm = 1.0f;

float torque_corner1 = 0.0f;
float torque_corner2 = 0.0f;
float torque_corner3 = 0.0f;
float torque_real_corner1 = 0.0f;
float torque_real_corner2 = 0.0f;
float torque_real_corner3 = 0.0f;
float theta_w_dot2 = 0.0f;
float theta_w_dot3 = 0.0f;

void corner_setup() {
    // Optional: capture the true "corner-up" reference here instead of
    // assuming identity -- read q_w/q_x/q_y/q_z once imu_setup() has run
    // and the cube is held steady on the target corner, then store into
    // qd_w/qd_x/qd_y/qd_z.
}

namespace {

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Attitude error = conjugate(qd) * q, vector part only, scaled by 2
// (small-angle approx). Flips sign on the short-way-around path so the
// error doesn't try to unwind through 360 deg.
void attitude_error(float &ex, float &ey, float &ez) {
    const float cw =  qd_w, cx = -qd_x, cy = -qd_y, cz = -qd_z; // conj(qd)
    const float ew  = cw*q_w - cx*q_x - cy*q_y - cz*q_z;
    const float evx = cw*q_x + cx*q_w + cy*q_z - cz*q_y;
    const float evy = cw*q_y - cx*q_z + cy*q_w + cz*q_x;
    const float evz = cw*q_z + cx*q_y - cy*q_x + cz*q_w;

    const float sign = (ew < 0.0f) ? -1.0f : 1.0f;
    ex = 2.0f * sign * evx;
    ey = 2.0f * sign * evy;
    ez = 2.0f * sign * evz;
}

} // namespace

void corner_loop() {
    float ex, ey, ez;
    attitude_error(ex, ey, ez);

    const float x[9] = {
        ex, ey, ez,
        gyro_x, gyro_y, gyro_z,
        theta_w_dot1, theta_w_dot2, theta_w_dot3,
    };

    float u[3] = {0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 3; ++row) {
        float acc = 0.0f;
        for (int col = 0; col < 9; ++col) acc += K[row][col] * x[col];
        u[row] = -acc;
    }

    torque_corner1 = clampf(u[0], -kMaxCornerTorqueNm, kMaxCornerTorqueNm);
    torque_corner2 = clampf(u[1], -kMaxCornerTorqueNm, kMaxCornerTorqueNm);
    torque_corner3 = clampf(u[2], -kMaxCornerTorqueNm, kMaxCornerTorqueNm);
}
