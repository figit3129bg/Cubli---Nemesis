#include <Arduino.h>
#include <MoteusTeensy.h>

// CAN-FD bus: Teensy 4.1 CAN3 (pins 30/31), 1 Mbps arbitration, matching the
// wiring used on the sibling teensy41_cubli project.
ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

Moteus controller(canBus, []() {
    Moteus::Options options;
    options.id = 1;
    return options;
}());

// feedforward_torque/maximum_torque/kp_scale/kd_scale default to kIgnore
// (not sent); send them as floats, matching the original position_format.
Moteus::PositionMode::Format commandFormat = []() {
    Moteus::PositionMode::Format format;
    format.feedforward_torque = Moteus::kFloat;
    format.maximum_torque     = Moteus::kFloat;
    format.kp_scale           = Moteus::kFloat;
    format.kd_scale           = Moteus::kFloat;
    return format;
}();

const float target_torque_Nm = 0.01f;
const float max_torque_Nm    = 0.01f;

Moteus::PositionMode::Command command;

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
    while (errorCode != 0) {
        Serial.print("CAN error 0x");
        Serial.println(errorCode, HEX);
        delay(1000);
    }

    controller.SetStop();  // clear any faults before starting

    // Command is built ONCE, outside the loop
    command.position           = NAN;
    command.velocity           = 20.0f;
    command.kp_scale           = 0.0f;
    command.kd_scale           = 1.0f;
    command.feedforward_torque = target_torque_Nm;
    command.maximum_torque     = max_torque_Nm;
}

void loop() {
    // Re-sends the SAME command struct as a heartbeat, matching the moteus
    // requirement of a fresh command within its watchdog timeout.
    const bool got_result = controller.SetPosition(command, &commandFormat);

    if (got_result) {
        const auto &v = controller.last_result().values;
        Serial.print(" vel=");
        Serial.print(v.velocity);
        Serial.print(" torque=");
        Serial.println(v.torque);
    } else {
        Serial.println("no response!");
    }

    delay(20);
}
