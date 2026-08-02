#include <Arduino.h>
#include "moteus_interface.h"
#include "imu_bmi270.h"

const double K1 = -909.2963;
const double K2 = -65.6971;
const double K3 = -1.0005;

double torqueCalc(double K1, double K2, double K3, double theta_b, double theta_b_dot, double theta_w_dot) {
  return K1*theta_b + K2*theta_b_dot + K3*theta_w_dot;
}

void setup() {
  Serial.begin(115200);
  imuSetup();
  moteusSetup();
}

void loop() {
   imuLoop();

  double theta_bE = 0.78539816339 - theta_b;
  double theta_b_dotE = 0 - theta_b_dot;
  double theta_w_dotE = 0 - theta_w_dot;
  double torque = torqueCalc(K1, K2, K3, theta_bE, theta_b_dotE, theta_w_dotE);
  moteusSendTorque(torque);
}
