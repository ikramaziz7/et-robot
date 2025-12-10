#include <stdio.h>
#include "movement.h"
#include "sensors.h"
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <unistd.h>

#define Sleep( msec ) usleep(( msec ) * 1000 )

int main(int argc, const char *argv[])
{
    printf("OS is fantastic!!\n");
    
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
    

    //Test moving functions
    // Move: 1=forward, -1=backward
    // Turn: 1=right, -1=left
    // move forward 5 seconds, turn until color sensor reads 5, move backward 5 seconds, turn left 3 seconds, turn right 3 seconds, turn left 360 degrees (motor degrees)
    move(1);
    Sleep(5000);
    stop_motors();
    turn_until_colorsensor(5);
    move(-1);
    Sleep(5000);
    stop_motors();
    turn(-1);
    Sleep(3000);
    stop_motors();
    turn(1);
    Sleep(3000);
    stop_motors();
    turn_left_degrees(360);
    

    
    printf("Done!\n");
    ev3_uninit();
    
    return 0;
}