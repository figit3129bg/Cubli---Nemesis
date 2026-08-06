// 3-state Kalman filter for the 2D cubli, ported from the MATLAB KF()
// function used during controller development.
//
// State vector: x = [theta_b, theta_b_dot, theta_w_dot]
// Input:        Tm = motor torque commanded on the previous control step
// Measurement:  z  = [theta_meas, theta_b_dot_meas, theta_w_dot_meas]  (H = I)
//
// The filter's internal state (xhat, P) persists across calls, mirroring
// MATLAB's `persistent` variables; it is lazily initialized on first use
// and can be reset with kalmanFilterReset().

#include "functions.h"

#include <cmath>

namespace {

// --- Minimal 3x3 / 3-vector helpers (no Eigen dependency needed for a
// fixed 3-state filter) -----------------------------------------------

using Mat3 = double[3][3];
using Vec3 = double[3];

void matMul(const Mat3 a, const Mat3 b, Mat3 out) {
    Mat3 tmp;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) sum += a[i][k] * b[k][j];
            tmp[i][j] = sum;
        }
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out[i][j] = tmp[i][j];
}

void matVecMul(const Mat3 a, const Vec3 v, Vec3 out) {
    Vec3 tmp;
    for (int i = 0; i < 3; ++i) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) sum += a[i][k] * v[k];
        tmp[i] = sum;
    }
    for (int i = 0; i < 3; ++i) out[i] = tmp[i];
}

void matTranspose(const Mat3 a, Mat3 out) {
    Mat3 tmp;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) tmp[j][i] = a[i][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out[i][j] = tmp[i][j];
}

void matAdd(const Mat3 a, const Mat3 b, Mat3 out) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out[i][j] = a[i][j] + b[i][j];
}

// General 3x3 inverse via the adjugate/determinant method.
bool matInverse(const Mat3 a, Mat3 out) {
    const double c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    const double c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    const double c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    const double det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    if (std::fabs(det) < 1e-15) return false;

    const double invDet = 1.0 / det;
    out[0][0] = c00 * invDet;
    out[1][0] = c01 * invDet;
    out[2][0] = c02 * invDet;
    out[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * invDet;
    out[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invDet;
    out[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * invDet;
    out[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invDet;
    out[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * invDet;
    out[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * invDet;
    return true;
}

// --- Filter state (equivalent to MATLAB's `persistent` block) --------

bool g_initialized = false;

Vec3 xhat;
Mat3 P;
Mat3 Ad;
Vec3 Bd;
Mat3 Qkf;
Mat3 Rkf;

double degToRad(double deg) { return deg * M_PI / 180.0; }

void initFilter() {
    xhat[0] = 0.0;
    xhat[1] = 0.0;
    xhat[2] = 0.0;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P[i][j] = 0.0;
    P[0][0] = 0.01;
    P[1][1] = 0.01;
    P[2][2] = 0.1;

    // Linearized cubli model, same physical constants as lqr.cpp's gain
    // derivation / the MATLAB source this was ported from.
    const double mb = 0.199;       // kg
    const double mw = 0.065;       // kg
    const double l  = 0.114551;    // m
    const double Ib = 0.0009084;   // kg*m^2
    const double Iw = 0.0002035;   // kg*m^2
    const double Cb = 0.00102;
    const double Cw = 0.00005;
    const double g  = 9.81;        // m/s^2
    const double Km = 0.021;

    const double denomB = Ib + mw * l * l;

    Mat3 A = {
        {0.0, 1.0, 0.0},
        {(mb * l + mw * l) * g / denomB, -Cb / denomB, Cw / denomB},
        {-(mb * l + mw * l) * g / denomB, Cb / denomB,
         -(Ib + Iw + mw * l * l) * Cw / (Iw * denomB)},
    };

    Vec3 B = {
        0.0,
        -Km / denomB,
        Km * (Ib + Iw + mw * l * l) / (Iw * denomB),
    };

    const double dt = 0.001;

    // Ad = I + A*dt, Bd = B*dt
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Ad[i][j] = (i == j ? 1.0 : 0.0) + A[i][j] * dt;
        }
        Bd[i] = B[i] * dt;
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Qkf[i][j] = 0.0;
    Qkf[0][0] = 1;
    Qkf[1][1] = 1;
    Qkf[2][2] = 1;  // trust the model over the noisy wheel-rate measurement

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Rkf[i][j] = 0.0;
    Rkf[0][0] = 5e-5;  // theta_meas placeholder
    Rkf[1][1] = 5e-5;                         // theta_b_dot_meas
    Rkf[2][2] = 3e-5;                           // theta_w_dot_meas

    g_initialized = true;
}

}  // namespace

void kalmanFilterReset() { g_initialized = false; }

void kalmanFilter(double Tm,
                   double theta_meas,
                   double theta_b_dot_meas,
                   double theta_w_dot_meas,
                   double &theta_hat,
                   double &theta_b_dot_hat,
                   double &theta_w_dot_hat) {
    if (!g_initialized) initFilter();

    // --- Prediction: x_pred = Ad*xhat + Bd*Tm, P_pred = Ad*P*Ad' + Qkf ---
    Vec3 x_pred;
    matVecMul(Ad, xhat, x_pred);
    for (int i = 0; i < 3; ++i) x_pred[i] += Bd[i] * Tm;

    Mat3 AdT, AdP, P_pred;
    matTranspose(Ad, AdT);
    matMul(Ad, P, AdP);
    matMul(AdP, AdT, P_pred);
    matAdd(P_pred, Qkf, P_pred);

    // --- Kalman gain: L = P_pred * (P_pred + Rkf)^-1   (H = I) ---
    Mat3 S, Sinv;
    matAdd(P_pred, Rkf, S);
    if (!matInverse(S, Sinv)) {
        // Singular innovation covariance (shouldn't happen with Rkf > 0);
        // fall back to the prediction only, matching a zero-gain update.
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) Sinv[i][j] = 0.0;
    }
    Mat3 L;
    matMul(P_pred, Sinv, L);

    // --- Update: xhat = x_pred + L*(z - x_pred) ---
    const Vec3 z = {theta_meas, theta_b_dot_meas, theta_w_dot_meas};
    Vec3 innovation;
    for (int i = 0; i < 3; ++i) innovation[i] = z[i] - x_pred[i];

    Vec3 correction;
    matVecMul(L, innovation, correction);
    for (int i = 0; i < 3; ++i) xhat[i] = x_pred[i] + correction[i];

    // Joseph-form covariance update for numerical stability, then force
    // symmetry to cancel floating-point drift, matching the MATLAB source.
    Mat3 ImL = {
        {1.0 - L[0][0], -L[0][1], -L[0][2]},
        {-L[1][0], 1.0 - L[1][1], -L[1][2]},
        {-L[2][0], -L[2][1], 1.0 - L[2][2]},
    };
    Mat3 ImLT, term1, term1ImLT, LT, LRkf, term2;
    matTranspose(ImL, ImLT);
    matMul(ImL, P_pred, term1);
    matMul(term1, ImLT, term1ImLT);

    matTranspose(L, LT);
    matMul(L, Rkf, LRkf);
    matMul(LRkf, LT, term2);

    matAdd(term1ImLT, term2, P);

    Mat3 PT;
    matTranspose(P, PT);
    matAdd(P, PT, P);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P[i][j] *= 0.5;

    // NOTE: the MATLAB source currently returns the raw measurements
    // rather than the filtered estimate (its `theta_hat = xhat(1)` etc.
    // lines are commented out) even though xhat/P still get updated every
    // call above. Mirrored here so behavior matches exactly; swap these
    // three lines for xhat[0]/xhat[1]/xhat[2] to actually use the filter.
    theta_hat       = theta_meas;
    theta_b_dot_hat = theta_b_dot_meas;
    theta_w_dot_hat = theta_w_dot_meas;
}
