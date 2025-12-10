#ifndef MOVEMENT_H
#define MOVEMENT_H

void move(int direction);  // 1 for forward, -1 for backward
void turn(int direction);  // -1 for left, 1 for right
void stop_motors();
void turn_left_degrees(int degrees);
void turn_until_colorsensor(int target_value);

#endif // MOVEMENT_H