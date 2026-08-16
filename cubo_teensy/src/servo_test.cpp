#include <Arduino.h>
#include <Servo.h>

// Standalone servo test, mirroring test/servo.cpp's servo_setup/servo_loop
// but with real setup()/loop() so it can be built and uploaded on its own
// (see env:servo_test in platformio.ini) without pulling in main.cpp's
// IMU/CAN/moteus setup.
//
// Press Ctrl+C in the serial monitor to stop the servo. `pio device
// monitor` (miniterm) uses Ctrl+] as its own quit key, so Ctrl+C isn't
// swallowed by the terminal -- it's sent through as a plain byte (ASCII
// ETX, 0x03), which is what we watch for below.

Servo myServo;
bool stopped = false;

void stopServo() {
    myServo.write(90);   // park at idle
    myServo.detach();    // stop sending pulses entirely
    Serial.println("Ctrl+C received -- servo stopped.");
    stopped = true;
}

// Waits up to ms milliseconds, polling for Ctrl+C the whole time so a stop
// request lands immediately instead of waiting out the current delay().
// Returns true if Ctrl+C was seen.
bool waitOrStop(unsigned long ms) {
    const unsigned long start = millis();
    while (millis() - start < ms) {
        if (Serial.available() > 0 && Serial.read() == 0x03) {
            return true;
        }
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    myServo.attach(0);   // your servo signal pin
    myServo.write(90);   // idle position at boot
    Serial.println("Servo set! Press Ctrl+C to stop.");
}

void loop() {
    if (stopped) {
        return;   // idle -- servo already parked and left alone
    }

    myServo.write(110);
    if (waitOrStop(150)) { stopServo(); return; }
    myServo.write(90);
    if (waitOrStop(150)) { stopServo(); return; }
}
