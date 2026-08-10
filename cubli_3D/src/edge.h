#ifndef EDGE_H
#define EDGE_H

extern float theta_w_dot1;      // reaction wheel rate, fed back from moteus
extern float prev_torque_edge;  // torque commanded last cycle; the KF's input Tm
extern float torque_edge;       // clamped control effort, sent to controller1
extern float torque_real_edge;  // unclamped control effort, for logging

void edge_loop();

#endif
