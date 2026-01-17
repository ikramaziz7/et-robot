#ifndef POSITION_H
#define POSITION_H

typedef struct RobotContext RobotContext;

typedef struct Pose {
    float x;        // cm
    float y;        // cm
    int theta;    // gyro angle
    int bearing; 
} Pose;

extern Pose mineral_positions[];
extern Pose box_offset;
extern Pose box_position;

float degrees_to_cm(int deg);
void update_pose(RobotContext *ctx, int d_left, int d_right, int gyro_deg);
int heading_to_target(Pose current, Pose target);
float distance_to_target(Pose current, Pose target);

// Utility functions
void print_pose(const Pose *p);
void print_distance_to_target(Pose current, Pose target);
int normalize_angle(int a);

#endif // POSITION_H