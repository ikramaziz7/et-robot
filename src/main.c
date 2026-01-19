#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "movement.h"
#include "sensors.h"
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <unistd.h>
#include "state_machine.h"
#include "position.h"
#include <stdlib.h>

#define Sleep( msec ) usleep(( msec ) * 1000 )

int main(int argc, const char *argv[])
{

    // when standing on the side of the field with the starting positions facing the field
    const char *starting_position = (argc > 2) ? argv[2] : NULL;
    if (starting_position) {
        printf("Starting Position: %s\n", starting_position);
        apply_starting_position(starting_position);
        printf("Box position set to: x=%.2f, y=%.2f\n", box_position.x, box_position.y);
    } else {
        printf("No starting position provided (using defaults)\n");
    }

    //speak("ET phone home");
    // Initialize EV3
    if (ev3_init() == -1) {
        printf("Failed to initialize EV3\n");
        return 1;
    }
    
    printf("EV3 initialized\n");
    printf("Waiting for motors to be detected...\n");
    
    while (ev3_tacho_init() < 1) Sleep(1000);    
    
    init_motors();
    initSensors();
    resetGyro();
    Sleep(5000);

    // Sometimes used for troubleshooting and testing
    if (strcmp(argv[1], "simple1") == 0) {
        printf("Mode selected: Simple 1 mineral\n");
        bool isMineralPickedUp = false;
        bool isMineralDropped = false;
        
        while (!isMineralDropped) {
            if (isMineralDropped) {
                break;
            }

            int distance = getDistanceCM();
            printf("Distance: %d\n", distance);
            if (!isMineralPickedUp) {
                move(1, 15);
                if (distance <= 4) {
                    isMineralPickedUp = true;
                    stop_motors();
                    control_grabber_degrees(GRABBER_CLOSEDIR, GRABBER_DEGREES);
                    Sleep(1000);  // Wait for grabber to close

                    // Then lift up
                    control_lift_arm_degrees(LIFT_LIFTDIR, LIFT_DEGREES);
                }
            }
            else if (isMineralPickedUp) {
                turn_degrees(400, 1);
                move(1, 15);
                if (distance <= 4) {
                    stop_motors();
                    control_grabber_degrees(GRABBER_CLOSEDIR * -1, GRABBER_DEGREES);
                    isMineralDropped = true;
                }
            }
        }
    } 
    else if (strcmp(argv[1], "state_machine") == 0) {
        printf("Mode selected: State Machine\n");
        RobotContext ctx;
        ctx.current_state = STATE_STARTUP;
        ctx.previous_state = STATE_STARTUP;
        ctx.state_entry_time_ms = millis();
        ctx.state_align_time_ms = millis();
        ctx.state_align_adjust_time_ms = millis();
        ctx.target_mineral_color = -1;
        ctx.pickup_retry_count = 0;
        ctx.match_running = 1;
        ctx.pose.x = 0.0f;
        ctx.pose.y = 0.0f;
        ctx.pose.theta = 0;
        ctx.dst.x = 0.0f;
        ctx.dst.y = 0.0f;
        ctx.dst.theta = 0;
        ctx.target_ore_index = 0;
        ctx.target_assigned = 0;
        ctx.aligned_home_initial = 0;

        printf("Starting FSM\n");

        int g1 = getGyroAngle();
        // Control loop
        int ticCounter = 0;
        while (ctx.match_running) {
            // get encoder values
            int d_left, d_right;
            get_encoder_deltas(&d_left, &d_right);
            
            // And gyro angle
            int gyro = getGyroAngle();

            // Use encoder values plus angle to update position
            update_pose(&ctx, d_left, d_right, gyro);
            
            // Only print out sometimes
            if (++ticCounter % 50 == 0) {
                if (ctx.current_state == STATE_EXPLORE) {
                    print_distance_to_target(ctx.pose, mineral_positions[ctx.target_ore_index]);
                }
                else if (ctx.current_state == STATE_RETURN_HOME) {
                    print_distance_to_target(ctx.pose, box_position);
                }
                print_pose(&ctx.pose);
            }

            update_state_machine(&ctx);

            Sleep(20);   // 50 Hz update rate
        }

        ev3_uninit();

        printf("Match ended, robot stopped\n");
    } 
    else {
        printf("Unknown argument: %s\n", argv[1]);
        return 1;
    }
    

    return 0;
}