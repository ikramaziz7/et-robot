#include <stdio.h>
#include <stdlib.h>
#include <sensors.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"


// WIN32/UNIX compatibility
#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif

#define COLOR_MAX 7
#define Sleep( msec ) usleep(( msec ) * 1000 )

uint8_t sn_gyro  = DESC_LIMIT;   // gyro sensor index
uint8_t sn_color = DESC_LIMIT;   // color sensor index
uint8_t sn_sonar = SENSOR__NONE_;
uint8_t sn_touch = SENSOR__NONE_;

int gyro_zero = 0;

int gyro_val;
int color_val;
int sonar_val;
int compass_val;

// Calibration constants
#define OBSTACLE_THRESHOLD_CM 20.0  // Default safe distance
#define SENSOR_READ_DELAY 200        // ms between sensor reads


/* ---------------- SENSOR READERS ---------------- */

char* getColor() {
    int val;

    if (sn_color == DESC_LIMIT) {
        printf("Color sensor not initialized\n");
        return COLOR_UNKNOWN;
    }

    if (!get_sensor_value(0, sn_color, &val) || val < 0 || val > COLOR_MAX) {
        printf("No color detected\n");
        return COLOR_UNKNOWN;
    }

    color_val = val;  // optional, if you still want to store last value

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

int get_compass_value(void) {
    if (!get_sensor_value(0, sn_compass, &compass_val)) return 0;
    return compass_val;
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

        if (ev3_sensor[i].type_inx == HT_NXT_COMPASS && sn_sonar == DESC_LIMIT) {
            sn_compass = i;
            printf("  -> Compass sensor registered\n");
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

/* ===================== TOUCH & ULTRASONIC FUNCTIONS ===================== */

/*
 * Function: init_ultrasonic
 * Description: Initialize ultrasonic (sonar) sensor
 * Returns: 0 on success, -1 on failure
 */
int init_ultrasonic() {
    // Search for ultrasonic sensor
    if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar, 0)) {
        printf("[INFO] Ultrasonic sensor found on sensor %d\n", sn_sonar);
        return 0;  // Success
    } else {
        printf("[ERROR] Ultrasonic sensor NOT found!\n");
        sn_sonar = SENSOR__NONE_;
        return -1;  // Failed
    }
}

/*
 * Function: read_ultrasonic_distance
 * Description: Read distance from ultrasonic sensor
 * Returns: Distance in mm (divide by 10 for cm), or -1 on error
 */
float read_ultrasonic_distance() {
    float value;
    
    // Check if sensor was initialized
    if (sn_sonar == SENSOR__NONE_) {
        printf("[ERROR] Ultrasonic sensor not initialized!\n");
        return -1.0;
    }
    
    // Read sensor value 
    if (!get_sensor_value0(sn_sonar, &value)) {
        printf("[ERROR] Failed to read ultrasonic sensor\n");
        return -1.0;
    }
    
    // Value is in mm, so divide by 10 for cm
    float distance_cm = value / 10.0;
    printf("[DEBUG] Ultrasonic distance: %.1f cm\n", distance_cm);
    return distance_cm;
}

/*
 * Function: detect_obstacle_ahead
 * Description: Check if obstacle within threshold
 * Parameters: threshold_cm - minimum safe distance
 * Returns: 1 if obstacle detected, 0 if clear, -1 on error
 */
int detect_obstacle_ahead(float threshold_cm) {
    float distance = read_ultrasonic_distance();
    
    if (distance < 0) {
        return -1;  // Sensor error
    }
    
    if (distance < threshold_cm) {
        printf("[WARNING] Obstacle detected at %.1f cm!\n", distance);
        return 1;  // Obstacle!
    }
    
    return 0;  // Path clear
}

/*
 * Function: init_touch_sensor
 * Description: Initialize touch sensor
 * Returns: 0 on success, -1 on failure
 */
int init_touch_sensor() {
    // Search for touch sensor
    if (ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)) {
        printf("[INFO] Touch sensor found on sensor %d\n", sn_touch);
        return 0;  // Success
    } else {
        printf("[ERROR] Touch sensor NOT found!\n");
        sn_touch = SENSOR__NONE_;
        return -1;  // Failed
    }
}

/*
 * Function: read_touch_sensor
 * Description: Read touch sensor state
 * Returns: 1 if pressed, 0 if not pressed, -1 on error
 */
int read_touch_sensor() {
    int val;
    
    // Check if sensor was initialized
    if (sn_touch == SENSOR__NONE_) {
        printf("[ERROR] Touch sensor not initialized!\n");
        return -1;
    }
    
    // Read sensor value 
    if (!get_sensor_value(0, sn_touch, &val)) {
        printf("[ERROR] Failed to read touch sensor\n");
        return -1;
    }
    
    // val is 0 (not pressed) or non-zero (pressed)
    int pressed = (val != 0) ? 1 : 0;
    if (pressed) {
        printf("[DEBUG] Touch sensor: PRESSED\n");
    } else {
        printf("[DEBUG] Touch sensor: released\n");
    }
    return pressed;
}

/*
 * Function: wait_for_touch
 * Description: Block until touch sensor is pressed
 */
void wait_for_touch() {
    printf("[INFO] Waiting for touch sensor press...\n");
    while (1) {
        int pressed = read_touch_sensor();
        if (pressed == 1) {
            printf("[INFO] Touch detected! Starting...\n");
            break;
        }
        Sleep(200);  // Check every 200ms 
    }
}

/* ===================== UNIFIED INITIALIZATION ===================== */

/*
 * Function: init_all_sensors
 * Description: Initialize all sensors (color, gyro, ultrasonic, touch)
 * Returns: 0 if all sensors found, negative value for each missing sensor
 */
int init_all_sensors() {
    int result = 0;
    
    printf("========== Initializing All Sensors ==========\n");
    
    // Initialize color and gyro 
    initSensors();
    
    // Initialize ultrasonic
    if (init_ultrasonic() != 0) {
        result -= 1;
    }
    
    // Initialize touch sensor
    if (init_touch_sensor() != 0) {
        result -= 2;
    }
    
    printf("========== Sensor Initialization Complete ==========\n");
    return result;
}