#include <Arduino.h>
#include <MoteusTeensy.h>

// CAN-FD bus: Teensy 4.1 CAN3 (pins 30/31), 1 Mbps arbitration, matching the
// wiring used on the sibling teensy41_cubli project.
ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// Moteus Controllers
Moteus controller1(canBus, []() {
    Moteus::Options options;
    options.id = 1;
    return options;
}());

//Moteus controller2(canBus, []() {
//    Moteus::Options options;
//    options.id = 2;
//    return options;
//}());

//Moteus controller3(canBus, []() {
//    Moteus::Options options;
//    options.id = 3;
//    return options;
//}());


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

// Test Parameters
const float target_torque_Nm = 0.01f;
const float max_torque_Nm    = 0.01f;

// Commands
Moteus::PositionMode::Command command1;
//Moteus::PositionMode::Command command2;
//Moteus::PositionMode::Command command3;

// SETUP
void setup() {
    Serial.begin(115200);
    while (!Serial) { 
        delay(10); 
    }

    const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
    while (errorCode != 0) {
        Serial.print("CAN error 0x");
        Serial.println(errorCode, HEX);
        delay(1000);
    }

    // Clear any faults before starting
    controller1.SetStop();  
//    controller2.SetStop(); 
//    controller3.SetStop(); 

    delay(500);

    // Build command for motor 1
    command1.position           = NAN;
    command1.velocity           = 20.0f;
    command1.kp_scale           = 0.0f;
    command1.kd_scale           = 1.0f;
    command1.feedforward_torque = target_torque_Nm;
    command1.maximum_torque     = max_torque_Nm;

    // Build command for motor 2
//    command2.position           = NAN;
//    command2.velocity           = 20.0f;
//    command2.kp_scale           = 0.0f;
//     command2.kd_scale           = 1.0f;
//     command2.feedforward_torque = target_torque_Nm;
//     command2.maximum_torque     = max_torque_Nm;

//     // Build command for motor 3
//     command3.position           = NAN;
//     command3.velocity           = 20.0f;
//     command3.kp_scale           = 0.0f;
//     command3.kd_scale           = 1.0f;
//     command3.feedforward_torque = target_torque_Nm;
//     command3.maximum_torque     = max_torque_Nm;
}

// LOOP
void loop(){

// MOTOR 1
Serial.println("Testing Moteus ID 1"); 
unsigned long start1 = millis(); 

while (millis() - start1 < 5000) {
    bool got_result = controller1.SetPosition(command1, &commandFormat);

    if (got_result) {
        const auto &v = controller1.last_result().values;
        Serial.print(" vel=");
        Serial.print(v.velocity);
        Serial.print(" torque=");
        Serial.println(v.torque);
    } else {
        Serial.println("no response!");
    }

    delay(20);
}
controller1.SetStop(); 
Serial.println("ID1 STOPPED"); 
delay(1000);

// // MOTOR 2
// Serial.println("Testing Moteus ID 2"); 
// unsigned long start2 = millis(); 

// while (millis() - start2 < 5000) {
//     bool got_result = controller2.SetPosition(command2, &commandFormat);

//     if (got_result) {
//         const auto &v = controller2.last_result().values;
//         Serial.print(" vel=");
//         Serial.print(v.velocity);
//         Serial.print(" torque=");
//         Serial.println(v.torque);
//     } else {
//         Serial.println("no response!");
//     }

//     delay(20);
// }
// controller2.SetStop(); 
// Serial.println("ID2 STOPPED"); 
// delay(1000);

// // MOTOR 3
// Serial.println("Testing Moteus ID 3"); 
// unsigned long start3 = millis(); 

// while (millis() - start3 < 5000) {
//     bool got_result = controller3.SetPosition(command3, &commandFormat);

//     if (got_result) {
//         const auto &v = controller3.last_result().values;
//         Serial.print(" vel=");
//         Serial.print(v.velocity);
//         Serial.print(" torque=");
//         Serial.println(v.torque);
//     } else {
//         Serial.println("no response!");
//     }

//     delay(20);
// }
// controller3.SetStop(); 
// Serial.println("ID3 STOPPED"); 
// delay(2000);


Serial.println("Test cycle completed.");
} 
//End void loop