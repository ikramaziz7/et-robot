#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

#define COLOR_MAX 7

/* ---------------- GLOBALS ---------------- */

const char *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };

uint8_t sn_gyro  = DESC_LIMIT;   // gyro sensor index
uint8_t sn_color = DESC_LIMIT;   // color sensor index

int gyro_zero = 0;

int gyro_val;
int color_val;

/* ---------------- SENSOR READERS ---------------- */

char* getColor() {
    int val;

    if (sn_color == DESC_LIMIT) {
        printf("Color sensor not initialized\n");
        val = 0;
    }

    if (!get_sensor_value(0, sn_color, &val) || val < 0 || val > COLOR_MAX) {
        printf("No color detected\n");
        val = 0;
    }

    printf("Color: %s\n", color[val]);
    return color[color_val];
}

void resetGyro(void) {
    if (sn_gyro == DESC_LIMIT)
        return;

    get_sensor_value(0, sn_gyro, &gyro_zero);
}

int getTurnError(int target_angle) {
    int raw_angle;
    int current_angle;
    int error;

    if (sn_gyro == DESC_LIMIT)
        return 0;

    if (!get_sensor_value(0, sn_gyro, &raw_angle))
        return 0;

    current_angle = raw_angle - gyro_zero;
    error = target_angle - current_angle;

    if (abs(error) < 2)
        error = 0;

    return error;
}

/* ---------------- SENSOR INIT ---------------- */

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

        /* Print info */
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

        /* --------- DETECT & SAVE SENSORS --------- */

        if (ev3_sensor[i].type_inx == LEGO_EV3_GYRO && sn_gyro == DESC_LIMIT) {
            sn_gyro = i;
            set_sensor_mode(sn_gyro, "GYRO-ANG");
            printf("  -> Gyro sensor registered\n");
        }

        if (ev3_sensor[i].type_inx == LEGO_EV3_COLOR && sn_color == DESC_LIMIT) {
            sn_color = i;
            set_sensor_mode(sn_color, "COL-COLOR");
            printf("  -> Color sensor registered\n");
        }
    }

    /* --------- VALIDATION --------- */

    if (sn_gyro == DESC_LIMIT)
        printf("WARNING: Gyro sensor not found\n");
    //else
    //     sleep(5);   // gyro calibration

    if (sn_color == DESC_LIMIT)
        printf("WARNING: Color sensor not found\n");

    printf("Out of here\n");
}
