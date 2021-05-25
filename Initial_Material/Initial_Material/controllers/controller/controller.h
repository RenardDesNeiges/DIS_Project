#ifndef CONTOLLER_H
#define CONTOLLER_H

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include "../localization_controller/odometry.h"


// returns a migration urge vector (as a pose structure), passed by a pointer
void migration_urge(pose_t *migration, pose_t robot, pose_t goal);

// returns a consensus controller vector (as a pose structure), passed by a pointer
void concensus_controller(pose_t *concensus, pose_t robot);

// returns a local avoidance vector (as a pose structure), passed by a pointer
void local_avoidance_controller(pose_t *local, pose_t robot);

// returns a local avoidance vector (as a pose structure), passed by a pointer
void unicylce_controller(double *omega, double *v, pose_t robot, pose_t goal, double ka, double kb, double kc);

// returns wheel speed from angular and linear velocity commands
void unicylce_to_wheels(double *w_left, double *w_right, double u_omega, double u_v, double l);

#endif