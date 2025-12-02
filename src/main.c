#include <stdio.h>
#include "movement.h"
#include "sensors.h"

int main(int argc, const char *argv[])
{
    printf("OS is fantastic!!\n");

    // Call placeholder functions
    move_forward();
    int sensor_val = read_sensor();
    printf("Sensor value: %d\n", sensor_val);

    // We need to implement this...

    return 0;
}