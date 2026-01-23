#ifndef SENSORS_H
#define SENSORS_H

typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_BLACK,
    COLOR_BLUE,
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_RED,
    COLOR_WHITE,
    COLOR_BROWN
} ColorType;

ColorType getColor();
int getGyroAngle(void);
void initSensors();
int isTouchPressed(void);     // 1=pressed, 0=not pressed
int get_compass_value(void);

int init_ultrasonic(void);
float read_ultrasonic_distance(void);
int detect_obstacle_ahead(float threshold_cm);
int init_touch_sensor(void);
int read_touch_sensor(void);
void wait_for_touch(void);
int init_all_sensors(void);


#endif // SENSORS_H