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

void move_forward() {
    uint8_t sn_left, sn_right;
    int max_speed;
    FLAGS_T state;
    
    printf("Running motors forward for 5 seconds...\n");
    
    // Find motors on ports A and B
    if (ev3_search_tacho_plugged_in(OUTPUT_A, 0, &sn_left, 0) &&
        ev3_search_tacho_plugged_in(OUTPUT_B, 0, &sn_right, 0)) {
        
        get_tacho_max_speed(sn_left, &max_speed);
        printf("Max speed = %d\n", max_speed);
        
        // Configure and run left motor (port A)
        set_tacho_stop_action_inx(sn_left, TACHO_COAST);
        set_tacho_speed_sp(sn_left, max_speed * 2 / 3);
        set_tacho_time_sp(sn_left, 5000);  // 5 seconds
        set_tacho_ramp_up_sp(sn_left, 2000);
        set_tacho_ramp_down_sp(sn_left, 2000);
        set_tacho_command_inx(sn_left, TACHO_RUN_TIMED);
        
        // Configure and run right motor (port B)
        set_tacho_stop_action_inx(sn_right, TACHO_COAST);
        set_tacho_speed_sp(sn_right, max_speed * 2 / 3);
        set_tacho_time_sp(sn_right, 5000);  // 5 seconds
        set_tacho_ramp_up_sp(sn_right, 2000);
        set_tacho_ramp_down_sp(sn_right, 2000);
        set_tacho_command_inx(sn_right, TACHO_RUN_TIMED);
        
        // Wait for motors to stop
        Sleep(100);
        do {
            get_tacho_state_flags(sn_left, &state);
        } while (state);
        
        printf("Motors stopped\n");
        
    } else {
        printf("Motors NOT found on ports A and B\n");
    }
}

void move_backward() {
    printf("Moving backward (placeholder)\n");
}

void turn_left() {
    printf("Turning left (placeholder)\n");
}

void turn_right() {
    printf("Turning right (placeholder)\n");
}

