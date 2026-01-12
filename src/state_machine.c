#include "state_machine.h"
#include "position.h"
#include "movement.h"
#include "sensors.h"
#include <stdio.h>
#include "time_utils.h"

#define START_SPEED 5
#define EXPLORE_SPEED      15  // percent of max speed
#define AVOID_SPEED      15  // percent of max speed
#define SONAR_THRESHOLD    15  // cm for obstacle
#define MINERAL_DIST_TARGET_THRESHOLD 2.5f
#define ALIGN_TARGET_INTERVAL 10000
#define ALIGN_TARGET_ADJUST_INTERVAL 3000
#define ALIGN_DEGREES_TOLERANCE 2
#define Sleep( msec ) usleep(( msec ) * 1000 )

static void state_startup(RobotContext *ctx);
static void state_explore(RobotContext *ctx);
static void state_approach_mineral(RobotContext *ctx);
static void state_pickup_mineral(RobotContext *ctx);
static void state_return_home(RobotContext *ctx);
static void state_drop_mineral(RobotContext *ctx);
static void state_avoid_obstacle(RobotContext *ctx);
static void state_end_match(RobotContext *ctx);

RobotState curr_moving_state;

static int zigzag_direction = 1;
int smooth_align_turn = 0;

static void state_startup(RobotContext *ctx) {
    /* Entry actions */
    if (ctx->state_just_entered) {
        printf("[STATE] STARTUP\n");
        move(1, START_SPEED);
        ctx->state_just_entered = 0;
        ctx->dst=mineral_positions[ctx->target_ore_index];
    }

    // For the test sessions, when the robot is in front of the object
    int color = getColor();
    if (color == COLOR_YELLOW || color == COLOR_BLUE || color == COLOR_RED) {
        ctx->target_mineral_color = color;
        stop_motors();
        change_state(ctx, STATE_APPROACH_MINERAL);
        return;
    }
    change_state(ctx, STATE_EXPLORE);

    //change_state(ctx, STATE_APPROACH_MINERAL);

    /* Transition condition */
    /*if (millis() - ctx->state_entry_time_ms > 2000) {
        stop_motors();
        change_state(ctx, STATE_ALIGN_TARGET_POSE);
    }*/    
}

void move_to_target_position(RobotContext *ctx, Pose src_pos, Pose dst_pos) {
    int heading = 0;
    heading = heading_to_target_coord(src_pos, dst_pos);

    if (!ctx->turn_active) {
        int direction = (heading >= 0) ? 1 : -1;
        int angle = abs(heading); // degrees
        if (angle < ALIGN_DEGREES_TOLERANCE) {
            return;
        }
        
        if (!smooth_align_turn) {
            start_turn(ctx, angle, direction, EXPLORE_SPEED);
        }
        else {
            start_move_with_arc(ctx, direction, EXPLORE_SPEED, angle);
        }
    } else {
        if (!smooth_align_turn) {
            update_turn(ctx);
        }
        else {
            update_move_with_arc(ctx);
        }
    }
}

static void state_align_target_pose(RobotContext *ctx) {
    if (ctx->state_just_entered) {
        printf("[STATE] ALIGN TARGET POSE\n");
        ctx->state_just_entered = 0;
        ctx->turn_active = 0;
        ctx->state_entry_time_ms = millis();
    }     

    int distance = getDistanceCM();
    if (distance > 0 && distance < SONAR_THRESHOLD) {
        stop_motors();
        ctx->previous_state = ctx->current_state;
        change_state(ctx, STATE_AVOID_OBSTACLE);
        return;
    }

    move_to_target_position(ctx, ctx->pose, ctx->dst);
    
    if (!ctx->turn_active) {
        if (curr_moving_state == STATE_RETURN_HOME) {
            change_state(ctx, STATE_RETURN_HOME);
        }
        else {
            change_state(ctx, STATE_EXPLORE);
        }
    }
}

static void state_explore(RobotContext *ctx) {
    /* Entry actions */
    if (ctx->state_just_entered) {
        printf("[STATE] EXPLORE\n");
        curr_moving_state = STATE_EXPLORE;
        move(1, EXPLORE_SPEED);   // start forward
        ctx->state_just_entered = 0;
        ctx->state_entry_time_ms = millis();
        ctx->dst=mineral_positions[ctx->target_ore_index];
        printf("dst: %f %f %f\n", ctx->dst.x, ctx->dst.y, ctx->dst.theta);
    }

    // Check for minerals
    int color = getColor();
    if (color == COLOR_YELLOW || color == COLOR_BLUE || color == COLOR_RED) {
        ctx->target_mineral_color = color;
        stop_motors();
        change_state(ctx, STATE_APPROACH_MINERAL);
        return;
    }

    // Check for obstacles
    int distance = getDistanceCM();
    if (distance > 0 && distance < SONAR_THRESHOLD) {
        stop_motors();
        ctx->previous_state = ctx->current_state;
        change_state(ctx, STATE_AVOID_OBSTACLE);
        return;
    }

    if (millis() - ctx->state_align_time_ms > ALIGN_TARGET_INTERVAL) {
        smooth_align_turn = 0;
        change_state(ctx, STATE_ALIGN_TARGET_POSE);
        ctx->state_align_time_ms = millis();
    }
    else if (millis() - ctx->state_align_adjust_time_ms > ALIGN_TARGET_ADJUST_INTERVAL) {
        smooth_align_turn = 1;
        change_state(ctx, STATE_ALIGN_TARGET_POSE);
        ctx->state_align_adjust_time_ms = millis();
    }

}

static void state_approach_mineral(RobotContext *ctx) {
    if (ctx->state_just_entered) {
        move(1, EXPLORE_SPEED);
        printf("[STATE] APPROACH MINERAL\n");
        ctx->state_just_entered = 0;
    }

    if (millis() - ctx->state_entry_time_ms >= 2000) {
        stop_motors();
        change_state(ctx, STATE_PICKUP_MINERAL);
    }
}

static void state_pickup_mineral(RobotContext *ctx) {
    if (ctx->state_just_entered) {
        printf("[STATE] PICKUP MINERAL\n");
        ctx->state_just_entered = 0;
        
        // Close grabber first
        control_grabber_degrees(1, 220);
        Sleep(1000);  // Wait for grabber to close
        
        // Then lift up
        control_lift_arm_degrees(1, 450);
        Sleep(1000);  // Wait for lift to complete
        
        // Now go back home
        change_state(ctx, STATE_RETURN_HOME);
    }
}
static void state_return_home(RobotContext *ctx) {
    if (ctx->state_just_entered) {
        printf("[STATE] RETURN HOME\n");
        curr_moving_state = STATE_RETURN_HOME;
        move(1, EXPLORE_SPEED);
        smooth_align_turn = 0;
        ctx->state_just_entered = 0;
        ctx->dst=box_position;
    }
    if (millis() - ctx->state_entry_time_ms > 20000) {
        stop_motors();
    }

    float distance_to_box = distance_to_target(ctx->pose, box_position);
    if (distance_to_box < MINERAL_DIST_TARGET_THRESHOLD) {
        change_state(ctx, STATE_DROP_MINERAL);
    }

    int distance = getDistanceCM();
    if (distance > 0 && distance < SONAR_THRESHOLD) {
        stop_motors();
        ctx->previous_state = ctx->current_state;
        change_state(ctx, STATE_AVOID_OBSTACLE);
        return;
    }


    if (millis() - ctx->state_align_time_ms > ALIGN_TARGET_INTERVAL) {
        smooth_align_turn = 0;
        change_state(ctx, STATE_ALIGN_TARGET_POSE);
        ctx->state_align_time_ms = millis();
    }
    else if (millis() - ctx->state_align_adjust_time_ms > ALIGN_TARGET_ADJUST_INTERVAL) {
        smooth_align_turn = 1;
        change_state(ctx, STATE_ALIGN_TARGET_POSE);
        ctx->state_align_adjust_time_ms = millis();
    }
}

static void state_drop_mineral(RobotContext *ctx) {
    if (ctx->state_just_entered) {
        printf("[STATE] DROP MINERAL\n");
        ctx->state_just_entered = 0;
        smooth_align_turn = 0;
        ctx->target_ore_index++;
        
        // Open grabber to release mineral
        control_grabber_degrees(-1, 180);
        Sleep(1000);  // Wait for mineral to drop
        
        // Lower lift arm back down
        control_lift_arm_degrees(-1, 415);
        Sleep(1000);
        
        // Move back slightly
        move(-1, EXPLORE_SPEED);
        Sleep(500);
        stop_motors();
        
        // Continue exploring
        change_state(ctx, STATE_EXPLORE);
    }
}

static void state_avoid_obstacle(RobotContext *ctx) {
    if (ctx->state_just_entered) {
        printf("[STATE] AVOID_OBSTACLE\n");
        ctx->state_just_entered = 0;
    }

    float dist_to_target = distance_to_target(ctx->pose, box_position);
    if (dist_to_target < MINERAL_DIST_TARGET_THRESHOLD) {
        change_state(ctx, STATE_APPROACH_MINERAL);
    }
    else {
        for (int i = 0; i < 100; i++) {
            move(-1, AVOID_SPEED);
        }
        change_state(ctx, STATE_ALIGN_TARGET_POSE);
    }
}

static void state_end_match(RobotContext *ctx) {

}

void update_state_machine(RobotContext *ctx) {
    switch (ctx->current_state) {
        case STATE_STARTUP:           state_startup(ctx); break;
        case STATE_ALIGN_TARGET_POSE: state_align_target_pose(ctx); break;
        case STATE_EXPLORE:           state_explore(ctx); break;
        case STATE_APPROACH_MINERAL:  state_approach_mineral(ctx); break;
        case STATE_PICKUP_MINERAL:    state_pickup_mineral(ctx); break;
        case STATE_RETURN_HOME:       state_return_home(ctx); break;
        case STATE_DROP_MINERAL:      state_drop_mineral(ctx); break;
        case STATE_AVOID_OBSTACLE:    state_avoid_obstacle(ctx); break;
        case STATE_END_MATCH:         state_end_match(ctx); break;
        default:                      change_state(ctx, STATE_END_MATCH); break;
    }
}

void change_state(RobotContext *ctx, RobotState new_state) {

    if (ctx->current_state == new_state)
        return;  // prevent unnecessary transitions

    ctx->previous_state = ctx->current_state;
    ctx->current_state = new_state;
    ctx->state_entry_time_ms = millis();
    ctx->state_just_entered = 1;

    printf("[FSM] %d -> %d\n", ctx->previous_state, ctx->current_state);
}
