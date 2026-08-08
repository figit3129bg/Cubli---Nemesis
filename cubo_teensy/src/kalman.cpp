#include <Arduino.h>
#include "kalman.h"

namespace {

struct Vec3 { float v[3]; };
struct Mat3 { float m[3][3]; };

Vec3 operator+(const Vec3 &a, const Vec3 &b) { return { {a.v[0]+b.v[0], a.v[1]+b.v[1], a.v[2]+b.v[2]} }; }
Vec3 operator-(const Vec3 &a, const Vec3 &b) { return { {a.v[0]-b.v[0], a.v[1]-b.v[1], a.v[2]-b.v[2]} }; }
Vec3 operator*(const Vec3 &a, float s)       { return { {a.v[0]*s, a.v[1]*s, a.v[2]*s} }; }

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

Mat3 operator*(const Mat3 &A, float s) {
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
            r.m[i][j] = 0.0f;
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
    r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0f;
    return r;
}

Mat3 diag3(float a, float b, float c) {
    Mat3 r = {};
    r.m[0][0] = a; r.m[1][1] = b; r.m[2][2] = c;
    return r;
}

Mat3 inverse(const Mat3 &A) {
    float det =
        A.m[0][0]*(A.m[1][1]*A.m[2][2] - A.m[1][2]*A.m[2][1]) -
        A.m[0][1]*(A.m[1][0]*A.m[2][2] - A.m[1][2]*A.m[2][0]) +
        A.m[0][2]*(A.m[1][0]*A.m[2][1] - A.m[1][1]*A.m[2][0]);
    float invDet = 1.0f / det;

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

} // namespace

void KF(float Tm, float theta_meas, float theta_b_dot_meas, float theta_w_dot_meas,
        float &theta_hat, float &theta_b_dot_hat, float &theta_w_dot_hat)
{
    // persistent xhat P Ad Bd H Qkf Rkf
    static Vec3 xhat;
    static Mat3 P, Ad, H, Qkf, Rkf;
    static Vec3 Bd;
    static bool initialized = false;

    if (!initialized)
    {
        xhat = { {0.0f, 0.0f, 0.0f} };
        P = diag3(0.01f, 0.01f, 0.1f);

        const float mb = 0.199f;      // kg
        const float mw = 0.065f;      // kg
        const float l  = 0.114551f;   // m
        const float Ib = 0.0009084f;  // kg m^2
        const float Iw = 0.0002035f;  // kg m^2
        const float Cb = 0.00102f;
        const float Cw = 0.00005f;
        const float g  = 9.81f;       // m/s^2
        const float Km = 0.021f;

        Mat3 A;
        A.m[0][0] = 0.0f;
        A.m[0][1] = 1.0f;
        A.m[0][2] = 0.0f;

        A.m[1][0] = (mb*l + mw*l)*g / (Ib + mw*l*l);
        A.m[1][1] = -Cb / (Ib + mw*l*l);
        A.m[1][2] = Cw / (Ib + mw*l*l);

        A.m[2][0] = -(mb*l + mw*l)*g / (Ib + mw*l*l);
        A.m[2][1] = Cb / (Ib + mw*l*l);
        A.m[2][2] = -(Ib + Iw + mw*l*l)*Cw / (Iw*(Ib + mw*l*l));

        Vec3 B;
        B.v[0] = 0.0f;
        B.v[1] = -Km / (Ib + mw*l*l);
        B.v[2] = Km*(Ib + Iw + mw*l*l) / (Iw*(Ib + mw*l*l));

        const float dt = 0.001f;

        Ad = identity3() + A * dt;
        Bd = B * dt;
        H = identity3();

        Qkf = diag3(1, 1e-4f, 1e-5f);   // lowered index 3 (wheel) from 1e-4 -> 1e-5: trust the model more for wheel state
        Rkf = diag3(
            radians(0.5f) * radians(0.5f),  // theta_meas placeholder (replace with logged value when you have it)
            5.8e-3f,                        // theta_b_dot_meas
            3e-2f                           // theta_w_dot_meas raised from 7.62e-3 -> trust the noisy wheel measurement less
        );

        initialized = true;
    }

    // Prediction
    Vec3 x_pred = Ad * xhat + Bd * Tm;
    Mat3 P_pred = Ad * P * transpose(Ad) + Qkf;

    // Measurement
    Vec3 z = { {theta_meas, theta_b_dot_meas, theta_w_dot_meas} };

    // Kalman Gain
    Mat3 L = (P_pred * transpose(H)) * inverse(H * P_pred * transpose(H) + Rkf);

    // Update
    xhat = x_pred + L * (z - H * x_pred);

    Mat3 I3 = identity3();
    Mat3 ImLH = I3 - L * H;
    P = ImLH * P_pred * transpose(ImLH) + L * Rkf * transpose(L);
    P = (P + transpose(P)) * 0.5f;  // force symmetry

    // theta_hat       = xhat.v[0];
    // theta_b_dot_hat = xhat.v[1];
    // theta_w_dot_hat = xhat.v[2];

    theta_hat       = theta_meas;
    theta_b_dot_hat = theta_b_dot_meas;
    theta_w_dot_hat = theta_w_dot_meas;
}
