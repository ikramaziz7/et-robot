#include <stdio.h>
#include <stdint.h>
#include "movement.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "brick.h"

#ifndef __WIN32__
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif

// Helper function to find motors on ports A and B
static int find_motors(uint8_t *sn_left, uint8_t *sn_right) {
    if (ev3_search_tacho_plugged_in(OUTPUT_A, 0, sn_left, 0) &&
        ev3_search_tacho_plugged_in(OUTPUT_B, 0, sn_right, 0)) {
        return 1;
    }
    printf("Motors NOT found on ports A and B\n");
    return 0;
}

// Move function
// direction: 1 for forward, -1 for backward
void move(int direction) {
    uint8_t sn_left, sn_right;
    int max_speed;
    
    if (!find_motors(&sn_left, &sn_right)) return;
    
    get_tacho_max_speed(sn_left, &max_speed);
    printf("Max speed = %d\n", max_speed);
    
    // Calculate speed based on direction
    // (-1 for backward, 1 for forward)
    int speed = (max_speed * 2 / 3) * direction;
    
    // Configure both motors
    set_tacho_speed_sp(sn_left, speed);
    set_tacho_speed_sp(sn_right, speed);
    
    // Run both motors forever (no time limit)
    set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
    
    printf("Motors running\n");
}

void stop_motors() {
    uint8_t sn_left, sn_right;
    
    if (!find_motors(&sn_left, &sn_right)) return;

    // Set stop action before stopping
    // TODO: could use TACHO_HOLD instead which holds motor position, TACHO_COAST does not actively breake
    // TODO: we could also pass this into the function as a parameter if it changes what we want
    set_tacho_stop_action_inx(sn_left, TACHO_BRAKE);  
    set_tacho_stop_action_inx(sn_right, TACHO_BRAKE);
    
    set_tacho_command_inx(sn_left, TACHO_STOP);
    set_tacho_command_inx(sn_right, TACHO_STOP);
    
    printf("Motors stopped\n");
}

// Turn function
// direction: -1 for left, 1 for right
void turn(int direction) {
    uint8_t sn_left, sn_right;
    int max_speed;
    
    if (!find_motors(&sn_left, &sn_right)) return;
    
    get_tacho_max_speed(sn_left, &max_speed);
    
    if (direction < 0) {  // Left turn
        // Left motor stopped, right motor forward
        set_tacho_speed_sp(sn_left, 0);
        set_tacho_speed_sp(sn_right, max_speed * 2 / 3);
    } else {  // Right turn
        // Left motor forward, right motor stopped
        set_tacho_speed_sp(sn_left, max_speed * 2 / 3);
        set_tacho_speed_sp(sn_right, 0);
    }
    
    set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
}


// Degrees is based on motor rotation, not robot turn degrees
//TODO: This depends on wheel diameter and wheel base which we can calibrate if wanted
void turn_left_degrees(int degrees) {
    uint8_t sn_left, sn_right;
    int max_speed;
    FLAGS_T state;
    
    printf("Turning left %d degrees...\n", degrees);
    
    if (!find_motors(&sn_left, &sn_right)) return;
    
    get_tacho_max_speed(sn_left, &max_speed);
    
    // Configure left motor - move backward (negative)
    set_tacho_stop_action_inx(sn_left, TACHO_BRAKE);
    set_tacho_speed_sp(sn_left, -(max_speed / 2));
    set_tacho_position_sp(sn_left, -degrees);
    
    // Configure right motor - move forward (positive)
    set_tacho_stop_action_inx(sn_right, TACHO_BRAKE);
    set_tacho_speed_sp(sn_right, max_speed / 2);
    set_tacho_position_sp(sn_right, degrees);
    
    // Start both motors to run to relative position
    set_tacho_command_inx(sn_left, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(sn_right, TACHO_RUN_TO_REL_POS);
    
    // Wait for turn to complete
    Sleep(100);
    do {
        get_tacho_state_flags(sn_left, &state);
    } while (state);
    
    printf("Turn left complete\n");
}


void turn_until_colorsensor(int target_value) {
    uint8_t sn_left, sn_right;
    uint8_t color_sensor;
    int max_speed;
    int sensor_value;
    
    printf("Turning until sensor reaches %d\n", target_value);
    
    // Find the color sensor
    color_sensor = sensor_search(LEGO_EV3_COLOR);
    
    if (color_sensor == SENSOR__NONE_) {
        printf("Color sensor not found!\n");
        return;
    }
    
    if (!find_motors(&sn_left, &sn_right)) return;
    
    get_tacho_max_speed(sn_left, &max_speed);
    
    // Start turning right
    set_tacho_speed_sp(sn_left, max_speed / 2); 
    set_tacho_speed_sp(sn_right, -(max_speed / 2));
    
    set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
    
    // Keep turning until sensor reaches target
    do {
        get_sensor_value0(color_sensor, &sensor_value);
        printf("Current sensor value: %d\n", sensor_value);
        // Check every 50ms
        Sleep(50);
    } while (sensor_value != target_value);
    
    // Stop turn
    stop_motors();
    
    printf("Target sensor value reached: %d\n", sensor_value);
}

// Control the lift arm motor (raise/lower conveyor belt)
// 1 to raise, -1 to lower, 0 to stop
// TODO: will need to determine if we just have a preset time after raising to stop
void control_lift_arm(int direction) {
    uint8_t sn_lift;
    int max_speed;
    
    // Search for motor on port C (orange motor)
    if (!ev3_search_tacho_plugged_in(OUTPUT_C, 0, &sn_lift, 0)) {
        printf("Lift arm motor NOT found on port C\n");
        return;
    }
    
    if (direction == 0) {
        // Stop the lift motor
        set_tacho_stop_action_inx(sn_lift, TACHO_HOLD);
        set_tacho_command_inx(sn_lift, TACHO_STOP);
        printf("Lift arm stopped\n");
        return;
    }
    
    get_tacho_max_speed(sn_lift, &max_speed);
    
    // Set speed (and direction)
    int speed = (max_speed / 2) * direction;
    set_tacho_speed_sp(sn_lift, speed);
    
    // Run the motor
    set_tacho_command_inx(sn_lift, TACHO_RUN_FOREVER);
    
    printf("Lift arm %s\n", direction > 0 ? "raising" : "lowering");
}

// Control the grabber motor (open/close)
// 1 to close, -1 to open, 0 to stop
void control_grabber(int direction) {
    uint8_t sn_grabber;
    int max_speed;
    
    // Search for motor on port D
    if (!ev3_search_tacho_plugged_in(OUTPUT_D, 0, &sn_grabber, 0)) {
        printf("Grabber motor NOT found on port D\n");
        return;
    }
    
    if (direction == 0) {
        // Stop the grabber motor
        set_tacho_stop_action_inx(sn_grabber, TACHO_HOLD);
        set_tacho_command_inx(sn_grabber, TACHO_STOP);
        printf("Grabber stopped\n");
        return;
    }
    
    get_tacho_max_speed(sn_grabber, &max_speed);
    
    // Set speed (and direction))
    int speed = (max_speed / 3) * direction;
    set_tacho_speed_sp(sn_grabber, speed);
    
    // Run the motor
    set_tacho_command_inx(sn_grabber, TACHO_RUN_FOREVER);
    
    printf("Grabber %s\n", direction > 0 ? "closing" : "opening");
}