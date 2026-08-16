#include <Arduino.h>
#include <Servo.h>

Servo myServo;
static bool servo_stopped = false;

void servo_setup(){
    myServo.attach(0);   // your servo signal pin
    myServo.write(90);     // idle position at boot
    Serial.println("Servo set!");
}

void servo_loop(){
    if (servo_stopped) {
        return;   // idle -- servo already parked and left alone
    }

    // Ctrl+C sends ASCII ETX (0x03) over serial; a serial monitor whose own
    // quit key isn't Ctrl+C (e.g. `pio device monitor`, which uses Ctrl+])
    // passes it straight through, so treat it as a stop command here.
    while (Serial.available() > 0) {
        if (Serial.read() == 0x03) {
            myServo.write(90);   // park at idle
            myServo.detach();    // stop sending pulses entirely
            Serial.println("Ctrl+C received -- servo stopped.");
            servo_stopped = true;
            return;
        }
    }

    myServo.write(110);
    delay(200);
    myServo.write(90);
}