#ifndef CONTOLLER_H
#define CONTOLLER_H

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include "../localization_controller/odometry.h"
#include "../communication/communication.h"

//initializes the proximity sensors
void init_prox_sensor();

// returns a migration urge vector (as a pose structure), passed by a pointer
void migration_urge(pose_t *migration, pose_t robot, pose_t goal);

// returns a consensus controller vector (as a pose structure), passed by a pointer
void consensus_controller(pose_t *consensus, pose_t robot_pose, pose_t *goal_pose, double kp, double ki, int robot_id, double* w);

// returns a reynold controller vector (as a pose structure), passed by a pointer
void reynolds_controller(pose_t *reynold, pose_t robot_pose, double w_cohesion, double w_dispersion, double w_consistency, int robot_id, double RULE2_RADIUS, double **relative_pos);

// returns a local avoidance vector (as a pose structure), passed by a pointer
void local_avoidance_controller(pose_t *local, pose_t robot, pose_t control);

// returns a local avoidance vector (as a pose structure), passed by a pointer
void unicycle_controller(double *omega, double *v, pose_t robot, pose_t goal, double ka, double kb, double kc);

// returns wheel speed from angular and linear velocity commands
void unicylce_to_wheels(double *w_left, double *w_right, double u_omega, double u_v, double l, double threshold);

void init_range_bearing_estimates(int* robot_id, pose_t* goal_pose);

void get_hyperparameters_from_supervisor(int* robot_id);


#endif
