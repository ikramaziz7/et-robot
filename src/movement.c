#include <stdio.h>
#include <stdint.h>
#include "movement.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "brick.h"
#include "state_machine.h"
#include <unistd.h>

#define Sleep( msec ) usleep(( msec ) * 1000 )

static uint8_t sn_left = 0xFF;
static uint8_t sn_right = 0xFF;
static uint8_t sn_lift = 0xFF;
static uint8_t sn_grabber = 0xFF;

// Initialize motors once
int init_motors(void) {
    if (ev3_search_tacho_plugged_in(OUTPUT_A, 0, &sn_left, 0) &&
        ev3_search_tacho_plugged_in(OUTPUT_D, 0, &sn_lift, 0) &&
        ev3_search_tacho_plugged_in(OUTPUT_C, 0, &sn_grabber, 0) &&
        ev3_search_tacho_plugged_in(OUTPUT_B, 0, &sn_right, 0)) {
        printf("Motors found: Left=C, Right=B\n");
        stop_motors();
        return 1;
    }
    printf("Motors NOT found on ports C and B\n");
    return 0;
}

void move(int direction, int speed_ratio) {
    if (sn_left == 0xFF || sn_right == 0xFF) {
        if (!init_motors()) return;
    }

    // quick fix because we reversed motors
    direction = -direction;

    if (speed_ratio < 0) speed_ratio = 0;
    if (speed_ratio > 100) speed_ratio = 100;

    int max_speed;
    get_tacho_max_speed(sn_left, &max_speed);

    int speed = (max_speed * speed_ratio / 100) * direction;

    set_tacho_speed_sp(sn_left, speed);
    set_tacho_speed_sp(sn_right, speed);

    set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
}


// Stop motors with HOLD for precision
void stop_motors(void) {
    if (sn_left == 0xFF || sn_right == 0xFF) {
        if (!init_motors()) return;
    }

    set_tacho_stop_action_inx(sn_left, TACHO_COAST);
    set_tacho_stop_action_inx(sn_right, TACHO_COAST);
    set_tacho_stop_action_inx(sn_grabber, TACHO_COAST);
    set_tacho_stop_action_inx(sn_lift, TACHO_COAST);
    set_tacho_command_inx(sn_left, TACHO_STOP);
    set_tacho_command_inx(sn_right, TACHO_STOP);
    set_tacho_command_inx(sn_grabber, TACHO_STOP);
    set_tacho_command_inx(sn_lift, TACHO_STOP);
}

// Turn on spot: -1=left, 1=right
void turn(int direction, int speed_ratio) {
    if (sn_left == 0xFF || sn_right == 0xFF) {
        if (!init_motors()) return;
    }

    int max_speed;
    get_tacho_max_speed(sn_left, &max_speed);
    int speed = max_speed * speed_ratio / 100;

    set_tacho_speed_sp(sn_left, -speed * direction);
    set_tacho_speed_sp(sn_right, speed * direction);

    set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
}


void turn_degrees(int degrees, int direction) {
    if (sn_left == 0xFF || sn_right == 0xFF) {
        if (!init_motors()) return;
    }

    int max_speed;
    get_tacho_max_speed(sn_left, &max_speed);

    set_tacho_stop_action_inx(sn_left, TACHO_HOLD);
    set_tacho_stop_action_inx(sn_right, TACHO_HOLD);

    set_tacho_speed_sp(sn_left,  (max_speed / 2) * direction);
    set_tacho_speed_sp(sn_right, (max_speed / 2) * -direction);

    set_tacho_position_sp(sn_left,  degrees * direction);
    set_tacho_position_sp(sn_right, degrees * -direction);

    set_tacho_command_inx(sn_left, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(sn_right, TACHO_RUN_TO_REL_POS);

    FLAGS_T state_left, state_right;

    /* Wait until BOTH motors are done */
    do {
        get_tacho_state_flags(sn_left,  &state_left);
        get_tacho_state_flags(sn_right, &state_right);
        Sleep(5);
    } while (state_left != 0 || state_right != 0);

    printf("Turn %s %d degrees complete\n",
           (direction < 0) ? "left" : "right",
           degrees);
}

/**
 * Turn robot in place by a specific angle
 * @param angle: motor rotation degrees (not robot degrees)
 * @param direction: -1=left, 1=right
 * @param speed_percent: 0-100 (% of max speed)
 */
void turn_degrees_speed(int angle, int direction, int speed_percent) {
    if (sn_left == 0xFF || sn_right == 0xFF) {
        if (!init_motors()) return;
    }

    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;

    int max_speed;
    get_tacho_max_speed(sn_left, &max_speed);
    int speed = max_speed * speed_percent / 100;

    // Set stop action
    set_tacho_stop_action_inx(sn_left, TACHO_HOLD);
    set_tacho_stop_action_inx(sn_right, TACHO_HOLD);

    // Set motor speeds: left and right turn in opposite directions
    set_tacho_speed_sp(sn_left, speed * direction);
    set_tacho_speed_sp(sn_right, speed * -direction);

    // Set relative position
    set_tacho_position_sp(sn_left, angle * direction);
    set_tacho_position_sp(sn_right, angle * -direction);

    // Run to relative position
    set_tacho_command_inx(sn_left, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(sn_right, TACHO_RUN_TO_REL_POS);

    // Wait until both motors stop
    FLAGS_T state_left, state_right;
    do {
        get_tacho_state_flags(sn_left, &state_left);
        get_tacho_state_flags(sn_right, &state_right);
        Sleep(5);
    } while (state_left != 0 || state_right != 0);

    
    printf("Turn %s %d degrees at %d%% speed complete\n",
           (direction < 0) ? "left" : "right",
           angle,
           speed_percent);
}

void start_move_with_arc(RobotContext *ctx,
                         int direction,
                         int speed_percent,
                         int target_degrees)
{
    direction = -direction;

    if (sn_left == 0xFF || sn_right == 0xFF) {
        if (!init_motors()) return;
    }

    if (direction != -1 && direction != 1) return;
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;

    int max_speed;
    get_tacho_max_speed(sn_left, &max_speed);

    /* Curve tuning */
    const int CURVE_DELTA = 40;  // increase for tighter arc

    int left_pct  = speed_percent;
    int right_pct = speed_percent;

    if (direction < 0) {
        left_pct -= CURVE_DELTA;          // curve left
        if (left_pct < 10) left_pct = 10;
    } else {
        right_pct -= CURVE_DELTA;         // curve right
        if (right_pct < 10) right_pct = 10;
    }

    /* ---- Store motion state ---- */
    ctx->turn_active         = 1;
    ctx->turn_direction      = direction;
    ctx->turn_speed_percent  = speed_percent;
    ctx->turn_target_degrees = target_degrees;
    ctx->turn_start_gyro     = getGyroAngle();

    /* ---- Forward motion with curvature ---- */
    set_tacho_speed_sp(sn_left,  -max_speed * left_pct  / 100);
    set_tacho_speed_sp(sn_right, -max_speed * right_pct / 100);

    set_tacho_command_inx(sn_left,  TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);

    printf("Arc move started: %s, speed=%d%%, target=%d°\n",
           (direction < 0) ? "left" : "right",
           speed_percent,
           target_degrees);
}

void update_move_with_arc(RobotContext *ctx)
{
    if (!ctx->turn_active) return;

    int current_gyro = getGyroAngle();
    int delta = abs(current_gyro - ctx->turn_start_gyro);

    if (delta >= ctx->turn_target_degrees) {
        stop_motors();
        ctx->turn_active = 0;

        printf("Arc move complete (gyro delta = %d°)\n", delta);
    }
}

void start_turn(RobotContext *ctx, int degrees, int direction, int speed_percent) {
    direction = -direction;

    if (sn_left == 0xFF || sn_right == 0xFF) {
        if (!init_motors()) return;
    }

    if (degrees <= 0) return;
    if (direction != -1 && direction != 1) return;

    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;

    int max_speed;
    get_tacho_max_speed(sn_left, &max_speed);
    int speed = max_speed * speed_percent / 100;

    /* Store turn state */
    ctx->turn_active         = 1;
    ctx->turn_direction      = direction;
    ctx->turn_speed_percent  = speed_percent;
    ctx->turn_target_degrees = degrees;
    ctx->turn_start_gyro     = getGyroAngle();

    /* Motor setup */
    set_tacho_stop_action_inx(sn_left,  TACHO_HOLD);
    set_tacho_stop_action_inx(sn_right, TACHO_HOLD);

    set_tacho_speed_sp(sn_left,  speed * direction);
    set_tacho_speed_sp(sn_right, speed * -direction);

    set_tacho_command_inx(sn_left,  TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);

    printf("Gyro turn started: %s %d deg at %d%%\n",
           (direction < 0) ? "left" : "right",
           degrees,
           speed_percent);
}

void update_turn(RobotContext *ctx) {
    if (!ctx->turn_active) return;

    int current_gyro = getGyroAngle();
    int delta = abs(current_gyro - ctx->turn_start_gyro);

    if (delta >= ctx->turn_target_degrees) {
        stop_motors();
        ctx->turn_active = 0;

        printf("Gyro turn complete (delta=%d)\n", delta);
    }
}

void get_encoder_deltas(int *d_left, int *d_right)
{
    static int last_left, last_right;
    static int initialized = 0;  // flag

    int cur_left, cur_right;
    get_tacho_position(sn_left, &cur_left);
    get_tacho_position(sn_right, &cur_right);

    if (!initialized) {
        last_left = cur_left;
        last_right = cur_right;
        *d_left = 0;
        *d_right = 0;
        initialized = 1;
        return;
    }

    *d_left  = cur_left  - last_left;
    *d_right = cur_right - last_right;

    last_left  = cur_left;
    last_right = cur_right;
}

// Control the lift arm motor (raise/lower conveyor belt)
// 1 to raise, -1 to lower, 0 to stop
// TODO: will need to determine if we just have a preset time after raising to stop
void control_lift_arm_degrees(int direction, int degrees) {
    int max_speed;

    // Direction sanity check
    if (direction != 1 && direction != -1) {
        printf("Invalid direction\n");
        return;
    }

    // Get max speed
    get_tacho_max_speed(sn_lift, &max_speed);
    set_tacho_speed_sp(sn_lift, max_speed / 2);
    // Set how far to move (degrees)
    set_tacho_position_sp(sn_lift, degrees * direction);
    // Optional: hold position after movement
    set_tacho_stop_action_inx(sn_lift, TACHO_HOLD);
    // Run motor to relative position
    set_tacho_command_inx(sn_lift, TACHO_RUN_TO_REL_POS);

    printf("Lift arm moving %d degrees %s\n",
           degrees,
           direction > 0 ? "up" : "down");
}



/**
 * Control grabber with specific degree rotation
 * @param direction: 1 to close, -1 to open
 * @param degrees: rotation amount (e.g., 180 degrees)
 */
void control_grabber_degrees(int direction, int degrees) {
    uint8_t sn_grabber;
    int max_speed;
    
    // Search for motor on port D (grabber motor)
    if (!ev3_search_tacho_plugged_in(OUTPUT_D, 0, &sn_grabber, 0)) {
        printf("Grabber motor NOT found on port D\n");
        return;
    }
    
    get_tacho_max_speed(sn_grabber, &max_speed);
    
    // Set stop action to hold position
    set_tacho_stop_action_inx(sn_grabber, TACHO_HOLD);
    
    // Set speed (adjust speed as needed - currently 1/3 of max)
    int speed = max_speed / 3;
    set_tacho_speed_sp(sn_grabber, speed);
    
    // Set position target (negative direction means open, positive means close)
    set_tacho_position_sp(sn_grabber, degrees * direction);
    
    // Execute movement
    set_tacho_command_inx(sn_grabber, TACHO_RUN_TO_REL_POS);
    
    // Wait until movement complete
    FLAGS_T state;
    do {
        get_tacho_state_flags(sn_grabber, &state);
        Sleep(10);
    } while (state != 0);
    
    printf("Grabber %s by %d degrees\n", 
           direction > 0 ? "closed" : "opened", degrees);
}

/**
 * Control lift arm with specific degree rotation
 * @param direction: 1 to raise, -1 to lower
 * @param degrees: rotation amount (e.g., 415 degrees to lift over box)
 */
void control_lift_arm_degrees(int direction, int degrees) {
    uint8_t sn_lift;
    int max_speed;
    
    // Search for motor on port A or C (lift arm motor)
    if (!ev3_search_tacho_plugged_in(OUTPUT_A, 0, &sn_lift, 0)) {
        printf("Lift arm motor NOT found on port A\n");
        return;
    }
    
    get_tacho_max_speed(sn_lift, &max_speed);
    
    // Set stop action to hold position
    set_tacho_stop_action_inx(sn_lift, TACHO_HOLD);
    
    // Set speed (half of max speed for controlled movement)
    int speed = max_speed / 2;
    set_tacho_speed_sp(sn_lift, speed);
    
    // Set position target
    set_tacho_position_sp(sn_lift, degrees * direction);
    
    // Execute movement
    set_tacho_command_inx(sn_lift, TACHO_RUN_TO_REL_POS);
    
    // Wait until movement complete
    FLAGS_T state;
    do {
        get_tacho_state_flags(sn_lift, &state);
        Sleep(10);
    } while (state != 0);
    
    printf("Lift arm %s by %d degrees\n", 
           direction > 0 ? "raised" : "lowered", degrees);
}

/**
 * Continuous control for grabber (for testing)
 * @param direction: 1 to close, -1 to open, 0 to stop
 */
void control_grabber(int direction) {
    uint8_t sn_grabber;
    int max_speed;
    
    if (!ev3_search_tacho_plugged_in(OUTPUT_D, 0, &sn_grabber, 0)) {
        printf("Grabber motor NOT found on port D\n");
        return;
    }
    
    if (direction == 0) {
        set_tacho_stop_action_inx(sn_grabber, TACHO_HOLD);
        set_tacho_command_inx(sn_grabber, TACHO_STOP);
        printf("Grabber stopped\n");
        return;
    }
    
    get_tacho_max_speed(sn_grabber, &max_speed);
    int speed = (max_speed / 3) * direction;
    set_tacho_speed_sp(sn_grabber, speed);
    set_tacho_command_inx(sn_grabber, TACHO_RUN_FOREVER);
    
    printf("Grabber %s\n", direction > 0 ? "closing" : "opening");
}

/**
 * Continuous control for lift arm (for testing)
 * @param direction: 1 to raise, -1 to lower, 0 to stop
 */
void control_lift_arm(int direction) {
    uint8_t sn_lift;
    int max_speed;
    
    if (!ev3_search_tacho_plugged_in(OUTPUT_A, 0, &sn_lift, 0)) {
        printf("Lift arm motor NOT found on port A\n");
        return;
    }
    
    if (direction == 0) {
        set_tacho_stop_action_inx(sn_lift, TACHO_HOLD);
        set_tacho_command_inx(sn_lift, TACHO_STOP);
        printf("Lift arm stopped\n");
        return;
    }
    
    get_tacho_max_speed(sn_lift, &max_speed);
    int speed = (max_speed / 2) * direction;
    set_tacho_speed_sp(sn_lift, speed);
    set_tacho_command_inx(sn_lift, TACHO_RUN_FOREVER);
    
    printf("Lift arm %s\n", direction > 0 ? "raising" : "lowering");
}