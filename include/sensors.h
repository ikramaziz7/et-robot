#ifndef SENSORS_H
#define SENSORS_H

char* getColor();
int getTurnError(int target_angle);
void initSensors();

int init_ultrasonic(void)
float read_ultrasonic_distance(void);
int detect_obstacle_ahead(float threshold_cm);
int init_touch_sensor(void);
int read_touch_sensor(void);
void wait_for_touch(void);
int init_all_sensors(void);


#endif // SENSORS_H