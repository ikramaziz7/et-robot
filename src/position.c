#include "position.h"
#include "sensors.h"
#include <math.h>
#include "state_machine.h"

#define M_PI 3.14159265358979323846
#define WHEEL_DIAMETER_CM 7.5
#define WHEEL_CIRC_CM (3.14159 * WHEEL_DIAMETER_CM)

// In cm
Pose mineral_positions[] = {
    {60.0f, 0.0f, 30, 320},   
    {60.0f, 35.0f, 30, 320},   
};

Pose box_position = {0.0f, 40.0f, 0, -1};

// Convert encoder degrees to cm
float degrees_to_cm(int deg) {
    return (deg / 360.0f) * WHEEL_CIRC_CM;
}

void update_pose(RobotContext *ctx, int d_left, int d_right, int gyro_deg)
{
    float dist_left  = -degrees_to_cm(d_left);
    float dist_right = -degrees_to_cm(d_right);

    float dist = (dist_left + dist_right) / 2.0f;

    //ctx->pose.bearing = get_compass_value();

    // Convert gyro degrees to radians
    float theta = gyro_deg * (M_PI / 180.0f);
    //float theta = ctx->pose.bearing * (M_PI / 180.0f);

    // Update position
    ctx->pose.x += dist * cosf(theta);
    ctx->pose.y += dist * sinf(theta);
    ctx->pose.theta = gyro_deg;
    
}

void print_pose(const Pose *p)
{
    if (!p) {
        printf("Pose: (null)\n");
        return;
    }

    printf("Pose -> X: %.2f cm | Y: %.2f cm | Theta: %d deg\n",
           p->x,
           p->y,
           p->theta);
}

void print_distance_to_target(Pose current, Pose target) {
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    float distance = sqrtf(dx*dx + dy*dy);
    printf("Distance to target: %f\n", distance);
}

float distance_to_target(Pose current, Pose target) {
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    return sqrtf(dx*dx + dy*dy);
}

int heading_to_target_coord(Pose current, Pose target) {
    float dx = target.x - current.x;
    float dy = target.y - current.y;

    // atan2f returns radians
    float desired_rad = atan2f(dy, dx);

    // convert to degrees
    int desired_deg = (int)(desired_rad * 180.0f / M_PI);

    // compute difference relative to current heading
    int diff = desired_deg - current.theta;

    return normalize_angle(diff);
}

int heading_to_target(Pose current, Pose target) {
    int diff = target.theta - current.theta;
    return normalize_angle(diff);
}

int heading_to_target_compass(Pose current, Pose target) {
    int diff = target.bearing - current.bearing;
    return normalize_angle(diff);
}

int normalize_angle(int a) {
    while (a > 180)  a -= 360;
    while (a < -180) a += 360;
    return a;
}