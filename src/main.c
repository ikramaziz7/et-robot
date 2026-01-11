#include <stdio.h>
#include "movement.h"
#include "sensors.h"
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <unistd.h>
#include "state_machine.h"
#include "position.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )

int main(int argc, const char *argv[])
{
    
    // Initialize EV3
    if (ev3_init() == -1) {
        printf("Failed to initialize EV3\n");
        return 1;
    }
    
    printf("EV3 initialized\n");
    printf("Waiting for motors to be detected...\n");
    
    // Wait for motors to be detected
    while (ev3_tacho_init() < 1) Sleep(1000);
    
    printf("Motors detected!\n");
    
    init_motors();
    initSensors();
    resetGyro();
    Sleep(5000);

    // // Init context struct
    
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

    srand(time(NULL));   // for random escape turns
    printf("Starting FSM\n");

    int g1 = getGyroAngle();
    /* ---------- 5. Main Control Loop ---------- */
    int ticCounter = 0;
    while (ctx.match_running) {
        int d_left, d_right;
        get_encoder_deltas(&d_left, &d_right);

        int gyro = getGyroAngle();
        update_pose(&ctx, d_left, d_right, gyro);

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

    return 0;
}