#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <cmath>

BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

float theta_b_dot, theta_b_dotX, theta_b_dotY, theta_b_dotZ, theta_b_accel, theta_b;
float theta_offset = 0.0f; // rad, the raw atan2 reading that we want to call "0"

// keeps result in (-pi, pi] even if we subtract an offset near the +-180 seam
float wrapToPi(float angle)
{
    while (angle > PI)  angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

void calibrateZero()
{
    Serial.println("Calibrating zero reference...");
    Serial.println("Hold the cube steady at the -135 deg position now.");
    delay(3000); // time to get into position

    const int N = 200;
    double sum = 0;
    for (int i = 0; i < N; i++)
    {
        imu.getSensorData();
        float raw = atan2(imu.data.accelY, imu.data.accelX);
        sum += raw;
        delay(5);
    }
    theta_offset = sum / N;

    Serial.print("theta_offset (rad): ");
    Serial.print(theta_offset, 4);
    Serial.print("  (deg): ");
    Serial.println(theta_offset * 180.0 / PI, 2);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for serial port to open

    Serial.println("BMI270 Example 1 - Basic Readings I2C");
    Wire.begin();
    while (imu.beginI2C(i2cAddress) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }
    Serial.println("BMI270 connected!");

    calibrateZero();
}

void loop()
{
    imu.getSensorData();

    theta_b_dotX = imu.data.gyroX;
    theta_b_dotY = imu.data.gyroY;
    theta_b_dotZ = imu.data.gyroZ;

    // Rotation rate around the Z-axis (converted from deg/s to rad/s)
    theta_b_dot = imu.data.gyroZ * PI / 180.0;

    // Tilt / orientation angle in the X-Y plane around the Z-axis
    theta_b_accel = atan2(imu.data.accelY, imu.data.accelX);

    // Zero-referenced angle: 0 now means "the position you calibrated at"
    theta_b = wrapToPi(theta_b_accel - theta_offset);

    // --- CSV output for the laptop-side parser: theta_b,theta_b_dot ---
    Serial.print(theta_b, 6);
    Serial.print(",");
    Serial.println(theta_b_dot, 6);

    delay(20); // Delay ~20ms to keep the terminal easily readable
}



/* #include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <cmath>

BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68

float theta_b_dot, theta_b_dotX, theta_b_dotY, theta_b_dotZ, theta_b_accel, theta_b;

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for serial port to open

    Serial.println("BMI270 Example 1 - Basic Readings I2C");
    Wire.begin();
    while(imu.beginI2C(i2cAddress) != BMI2_OK)
    {
        Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
        delay(1000);
    }
    Serial.println("BMI270 connected!");
}

void loop()
{
    imu.getSensorData();
    
    theta_b_dotX = imu.data.gyroX;
    theta_b_dotY = imu.data.gyroY;
    theta_b_dotZ = imu.data.gyroZ;
    
    // Rotation rate around the Z-axis (converted from deg/s to rad/s)
    theta_b_dot = imu.data.gyroZ * PI / 180.0; 
    
    // Tilt / orientation angle in the X-Y plane around the Z-axis
    theta_b_accel = atan2(imu.data.accelY, imu.data.accelX); 
    
    theta_b = theta_b_accel;

    // --- Terminal Print Code ---
    Serial.print("Gyro Z (rad/s): ");
    Serial.print(theta_b_dot, 4);
    Serial.print(" | Angle (rad): ");
    Serial.print(theta_b, 4);
    Serial.print(" | Angle (deg): ");
    Serial.println(theta_b * 180.0 / PI, 2);

    delay(20); // Delay ~50ms to keep the terminal easily readable
}
 */
/* #include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

BMI270 imu;
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;

// ---- AXIS SELECTION (fill in after the hand-tilt test) ----
// theta_dot comes straight from whichever gyro axis is aligned with the
// cube's tipping edge. Example: if gyroX is the one that moves -> use imu.data.gyroX
//
// theta comes from atan2 of the two accel axes that lie in the tipping plane.
// Example placeholder below assumes Y and Z trade off as you tip it;
// SWAP THESE once you confirm which two axes actually respond.
#define GYRO_AXIS(imu)   (imu.data.gyroZ)      // deg/s -> TODO confirm
#define ACCEL_A1(imu)    (imu.data.accelX)     // TODO confirm
#define ACCEL_A2(imu)    (imu.data.accelY)     // TODO confirm
// -------------------------------------------------------------

const float DEG2RAD = 3.14159265f / 180.0f;

// Complementary filter weight: how much we trust the gyro-integrated angle
// vs the accel angle, per update. Closer to 1 = trust gyro more (drifts less
// correction, smoother). Closer to 0 = trust accel more (noisier, no drift).
const float ALPHA = 0.98f;

float gyroBias = 0.0f;   // deg/s, measured at startup while stationary
float theta = 0.0f;      // rad, fused angle estimate
unsigned long lastMicros = 0;

void calibrateGyroBias() {
    Serial.println("Calibrating gyro bias — keep the cube still...");
    const int N = 500;
    double sum = 0;
    for (int i = 0; i < N; i++) {
        imu.getSensorData();
        sum += GYRO_AXIS(imu);
        delay(2);
    }
    gyroBias = sum / N;
    Serial.print("Gyro bias (deg/s): ");
    Serial.println(gyroBias, 4);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin(21, 22);

    Serial.println("Connecting to BMI270...");
    while (imu.beginI2C(i2cAddress) != BMI2_OK) {
        Serial.println("BMI270 not responding - check wiring/address");
        delay(1000);
    }
    Serial.println("BMI270 connected.");

    calibrateGyroBias();

    // Initialize theta from the accel reading so we don't start at a wrong angle
    imu.getSensorData();
    float a1 = ACCEL_A1(imu);
    float a2 = ACCEL_A2(imu);
    theta = atan2(a1, a2);

    lastMicros = micros();
}

void loop() {
    imu.getSensorData();

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;

    // theta_dot: bias-corrected gyro reading, converted to rad/s
    float gyro_dps = GYRO_AXIS(imu) - gyroBias;
    float theta_dot = gyro_dps * DEG2RAD;

    // theta_acc: instantaneous tilt estimate from accelerometer (noisy, no drift)
    float a1 = ACCEL_A1(imu);
    float a2 = ACCEL_A2(imu);
    float theta_acc = atan2(a1, a2);

    // Complementary filter: integrate gyro, pull toward accel estimate
    theta = ALPHA * (theta + theta_dot * dt) + (1.0f - ALPHA) * theta_acc;

    // Output: timestamp_us, theta (rad), theta_dot (rad/s)
    Serial.print(now); Serial.print(",");
    Serial.print(theta, 6); Serial.print(",");
    Serial.println(theta_dot, 6);

    delay(5); // ~200 Hz; tighten/loosen once you see real loop timing on the laptop side
} */


