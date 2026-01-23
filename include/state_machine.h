#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include "position.h"

typedef enum {
    STATE_STARTUP = 0,
    STATE_ALIGN_TARGET_POSE,
    STATE_EXPLORE,
    STATE_APPROACH_MINERAL,
    STATE_PICKUP_MINERAL,
    STATE_RETURN_HOME,
    STATE_DROP_MINERAL,
    STATE_AVOID_OBSTACLE,
    STATE_END_MATCH
} RobotState;


typedef struct RobotContext {
    RobotState current_state;
    RobotState previous_state;

    uint32_t state_entry_time_ms;
    uint32_t state_align_time_ms;
    uint32_t state_align_adjust_time_ms;
    int state_just_entered;

    int target_mineral_color;
    int pickup_retry_count;

    int match_running;

    int turn_active;          // 1 = currently turning
    int turn_target_degrees;  // how many degrees to turn
    int turn_direction;       // -1 = left, 1 = right
    int turn_speed_percent;   // 0-100
    int turn_start_left_pos;  // encoder values at start
    int turn_start_right_pos;
    int turn_start_gyro;          // gyro angle at start
    int aligned_home_initial;

    int target_ore_index;
    int target_assigned;

    Pose pose;
    Pose dst;
} RobotContext;


void update_state_machine(RobotContext *ctx);
void change_state(RobotContext *ctx, RobotState new_state);

#endif