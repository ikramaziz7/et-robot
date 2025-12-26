#ifndef MOVEMENT_H
#define MOVEMENT_H

void move(int direction);  // 1 for forward, -1 for backward
void turn(int direction);  // -1 for left, 1 for right
void stop_motors();
void turn_left_degrees(int degrees);
void turn_until_colorsensor(int target_value);
void control_lift_arm(int direction);  // 1 to raise, -1 to lower, 0 to stop
void control_grabber(int direction);   // 1 to close, -1 to open, 0 to stop

#endif // MOVEMENT_H