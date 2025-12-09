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
    
    // Move forward for 5 seconds
    move_forward();
    
    printf("Done!\n");
    ev3_uninit();
    
    return 0;
}