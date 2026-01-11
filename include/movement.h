#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <stdint.h>
#include "state_machine.h"

void move(int direction, int speed_ratio);  // 1 for forward, -1 for backward
void turn(int direction, int speed_ratio);  // -1 for left, 1 for right
void stop_motors();
void turn_left_degrees(int degrees);
void turn_until_colorsensor(int target_value);
void control_lift_arm_degrees(int direction, int degrees);  // 1 to raise, -1 to lower, 0 to stop
void control_grabber_degrees(int direction, int degrees);   // 1 to close, -1 to open, 0 to stop
void turn_degrees_speed(int angle, int direction, int speed_percent);
void start_move_with_arc(RobotContext *ctx, int direction, int speed_percent, int target_degrees);
void update_move_with_arc(RobotContext *ctx);
void update_turn(RobotContext *ctx);
void start_turn(RobotContext *ctx, int degrees, int direction, int speed_percent);
void get_encoder_deltas(int *d_left, int *d_right); // For odemtry

#endif // MOVEMENT_H