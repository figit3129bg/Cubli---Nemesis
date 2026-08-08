#include "functions.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
double radians(double deg) { return deg * kPi / 180.0; }

struct Vec3 { double v[3]; };
struct Mat3 { double m[3][3]; };

Vec3 operator+(const Vec3 &a, const Vec3 &b) { return { {a.v[0]+b.v[0], a.v[1]+b.v[1], a.v[2]+b.v[2]} }; }
Vec3 operator-(const Vec3 &a, const Vec3 &b) { return { {a.v[0]-b.v[0], a.v[1]-b.v[1], a.v[2]-b.v[2]} }; }
Vec3 operator*(const Vec3 &a, double s)      { return { {a.v[0]*s, a.v[1]*s, a.v[2]*s} }; }

Mat3 operator+(const Mat3 &A, const Mat3 &B) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = A.m[i][j] + B.m[i][j];
    return r;
}

Mat3 operator-(const Mat3 &A, const Mat3 &B) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = A.m[i][j] - B.m[i][j];
    return r;
}

Mat3 operator*(const Mat3 &A, double s) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = A.m[i][j] * s;
    return r;
}

Vec3 operator*(const Mat3 &A, const Vec3 &x) {
    Vec3 r;
    for (int i = 0; i < 3; i++)
        r.v[i] = A.m[i][0]*x.v[0] + A.m[i][1]*x.v[1] + A.m[i][2]*x.v[2];
    return r;
}

Mat3 operator*(const Mat3 &A, const Mat3 &B) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = 0.0;
            for (int k = 0; k < 3; k++)
                r.m[i][j] += A.m[i][k] * B.m[k][j];
        }
    return r;
}

Mat3 transpose(const Mat3 &A) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[j][i] = A.m[i][j];
    return r;
}

Mat3 identity3() {
    Mat3 r = {};
    r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
    return r;
}

Mat3 diag3(double a, double b, double c) {
    Mat3 r = {};
    r.m[0][0] = a; r.m[1][1] = b; r.m[2][2] = c;
    return r;
}

Mat3 inverse(const Mat3 &A) {
    double det =
        A.m[0][0]*(A.m[1][1]*A.m[2][2] - A.m[1][2]*A.m[2][1]) -
        A.m[0][1]*(A.m[1][0]*A.m[2][2] - A.m[1][2]*A.m[2][0]) +
        A.m[0][2]*(A.m[1][0]*A.m[2][1] - A.m[1][1]*A.m[2][0]);
    double invDet = 1.0 / det;

    Mat3 inv;
    inv.m[0][0] =  (A.m[1][1]*A.m[2][2] - A.m[1][2]*A.m[2][1]) * invDet;
    inv.m[0][1] = -(A.m[0][1]*A.m[2][2] - A.m[0][2]*A.m[2][1]) * invDet;
    inv.m[0][2] =  (A.m[0][1]*A.m[1][2] - A.m[0][2]*A.m[1][1]) * invDet;
    inv.m[1][0] = -(A.m[1][0]*A.m[2][2] - A.m[1][2]*A.m[2][0]) * invDet;
    inv.m[1][1] =  (A.m[0][0]*A.m[2][2] - A.m[0][2]*A.m[2][0]) * invDet;
    inv.m[1][2] = -(A.m[0][0]*A.m[1][2] - A.m[0][2]*A.m[1][0]) * invDet;
    inv.m[2][0] =  (A.m[1][0]*A.m[2][1] - A.m[1][1]*A.m[2][0]) * invDet;
    inv.m[2][1] = -(A.m[0][0]*A.m[2][1] - A.m[0][1]*A.m[2][0]) * invDet;
    inv.m[2][2] =  (A.m[0][0]*A.m[1][1] - A.m[0][1]*A.m[1][0]) * invDet;
    return inv;
}

// Persistent filter state (equivalent to the MATLAB version's "persistent"
// locals). Held in a resettable struct, rather than function-local statics,
// so kalmanFilterReset() has something to clear.
struct KalmanState {
    Vec3 xhat{};
    Mat3 P{}, Ad{}, H{}, Qkf{}, Rkf{};
    Vec3 Bd{};
    bool initialized = false;
};

KalmanState g_state;

} // namespace

void kalmanFilterReset() {
    g_state = KalmanState{};
}

void kalmanFilter(double Tm,
                   double theta_b_meas,
                   double theta_b_dot_meas,
                   double theta_w_dot_meas,
                   double &theta_b_hat,
                   double &theta_b_dot_hat,
                   double &theta_w_dot_hat)
{
    KalmanState &s = g_state;

    if (!s.initialized)
    {
        s.xhat = { {0.0, 0.0, 0.0} };
        s.P = diag3(0.01, 0.01, 0.1);

        const double mb = 0.199;      // kg
        const double mw = 0.065;      // kg
        const double l  = 0.114551;   // m
        const double Ib = 0.0009084;  // kg m^2
        const double Iw = 0.0002035;  // kg m^2
        const double Cb = 0.00102;
        const double Cw = 0.00005;
        const double g  = 9.81;       // m/s^2
        const double Km = 0.021;

        Mat3 A;
        A.m[0][0] = 0.0;
        A.m[0][1] = 1.0;
        A.m[0][2] = 0.0;

        A.m[1][0] = (mb*l + mw*l)*g / (Ib + mw*l*l);
        A.m[1][1] = -Cb / (Ib + mw*l*l);
        A.m[1][2] = Cw / (Ib + mw*l*l);

        A.m[2][0] = -(mb*l + mw*l)*g / (Ib + mw*l*l);
        A.m[2][1] = Cb / (Ib + mw*l*l);
        A.m[2][2] = -(Ib + Iw + mw*l*l)*Cw / (Iw*(Ib + mw*l*l));

        Vec3 B;
        B.v[0] = 0.0;
        B.v[1] = -Km / (Ib + mw*l*l);
        B.v[2] = Km*(Ib + Iw + mw*l*l) / (Iw*(Ib + mw*l*l));

        const double dt = 0.001;

        s.Ad = identity3() + A * dt;
        s.Bd = B * dt;
        s.H = identity3();

        s.Qkf = diag3(1e-6, 1e-4, 1e-5);
        s.Rkf = diag3(
            radians(0.5) * radians(0.5),  // theta_b_meas placeholder (replace with logged value when you have it)
            5.8e-3,                       // theta_b_dot_meas
            3e-2                          // theta_w_dot_meas
        );

        s.initialized = true;
    }

    // Prediction
    Vec3 x_pred = s.Ad * s.xhat + s.Bd * Tm;
    Mat3 P_pred = s.Ad * s.P * transpose(s.Ad) + s.Qkf;

    // Measurement
    Vec3 z = { {theta_b_meas, theta_b_dot_meas, theta_w_dot_meas} };

    // Kalman gain
    Mat3 L = (P_pred * transpose(s.H)) * inverse(s.H * P_pred * transpose(s.H) + s.Rkf);

    // Update
    s.xhat = x_pred + L * (z - s.H * x_pred);

    Mat3 I3 = identity3();
    Mat3 ImLH = I3 - L * s.H;
    s.P = ImLH * P_pred * transpose(ImLH) + L * s.Rkf * transpose(L);
    s.P = (s.P + transpose(s.P)) * 0.5;  // force symmetry

    // Pass-through, matching src/kalman.cpp's KF() until the estimate is
    // validated against logged data.
    theta_b_hat     = theta_b_meas;
    theta_b_dot_hat = theta_b_dot_meas;
    theta_w_dot_hat = theta_w_dot_meas;
}
