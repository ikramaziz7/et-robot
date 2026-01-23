#include <stdio.h>
#include <stdlib.h>
#include <sensors.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )
#define COLOR_MAX 7

uint8_t sn_gyro  = SENSOR__NONE_;   // gyro sensor index
uint8_t sn_color = SENSOR__NONE_;   // color sensor index
uint8_t sn_sonar = SENSOR__NONE_;
uint8_t sn_touch = SENSOR__NONE_;

int gyro_zero = 0;

int gyro_val;
int color_val;
int sonar_val;
int compass_val;

ColorType getColor() {
    int val;

    if (sn_color == DESC_LIMIT) {
        printf("Color sensor not initialized\n");
        return COLOR_UNKNOWN;
    }

    if (!get_sensor_value(0, sn_color, &val) || val < 0 || val > COLOR_MAX) {
        printf("No color detected\n");
        return COLOR_UNKNOWN;
    }

    color_val = val;

    //printf("Color detected: %d\n", val);  // optional debug
    return (ColorType)val;
}

void resetGyro(void) {
    if (sn_gyro == DESC_LIMIT)
        return;

    get_sensor_value(0, sn_gyro, &gyro_zero);
}

int getGyroAngle(void) {
    int raw;
    if (!get_sensor_value(0, sn_gyro, &raw)) return 0;
    return raw - gyro_zero;
}


int getDistanceCM(void) {
    int val;

    if (sn_sonar == DESC_LIMIT) {
        printf("Sonar sensor not initialized\n");
        return -1;
    }

    if (!get_sensor_value(0, sn_sonar, &val)) {
        printf("Failed to read sonar sensor\n");
        return -1;
    }

    int distance_cm = val / 10;   // convert mm → cm
    //printf("Distance: %d cm\n", distance_cm);

    return distance_cm;
}

// Initialize sensors
void initSensors(void) {
    int i;
    char s[256];
    int val;
    uint32_t n, ii;

    ev3_sensor_init();

    printf("Found sensors:\n");

    for (i = 0; i < DESC_LIMIT; i++) {

        if (ev3_sensor[i].type_inx == SENSOR_TYPE__NONE_)
            continue;

        printf("  type = %s\n", ev3_sensor_type(ev3_sensor[i].type_inx));
        printf("  port = %s\n", ev3_sensor_port_name(i, s));

        if (get_sensor_mode(i, s, sizeof(s)))
            printf("  mode = %s\n", s);

        if (get_sensor_num_values(i, &n)) {
            for (ii = 0; ii < n; ii++) {
                if (get_sensor_value(ii, i, &val))
                    printf("  value%d = %d\n", ii, val);
            }
        }

        // Detect sensors and callibrate gyro
        if (ev3_sensor[i].type_inx == LEGO_EV3_GYRO && sn_gyro == DESC_LIMIT) {
            sn_gyro = i;
            set_sensor_mode(sn_gyro, "GYRO-CAL");
            Sleep(500); // robot must be stationary

            set_sensor_mode(sn_gyro, "GYRO-ANG");
            Sleep(50);

            printf("  -> Gyro sensor callibrated and registered\n");
        }

        if (ev3_sensor[i].type_inx == LEGO_EV3_COLOR && sn_color == DESC_LIMIT) {
            sn_color = i;
            set_sensor_mode(sn_color, "COL-COLOR");
            printf("  -> Color sensor registered\n");
        }

        if (ev3_sensor[i].type_inx == LEGO_EV3_US && sn_sonar == DESC_LIMIT) {
            sn_sonar = i;
            set_sensor_mode(sn_sonar, "US-DIST-CM");
            printf("  -> Sonar sensor registered\n");
        }

    }

    /* --------- VALIDATION --------- */

    if (sn_gyro == DESC_LIMIT)
        printf("WARNING: Gyro sensor not found\n");

    if (sn_color == DESC_LIMIT)
        printf("WARNING: Color sensor not found\n");

    if (sn_sonar == DESC_LIMIT)
        printf("WARNING: Sonar sensor not found\n");

    printf("Sensor initialization complete.\n");
}
