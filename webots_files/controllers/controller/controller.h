/*****************************************************************************/
/* File:         controller.h                             		             */
/* Date:         2021	   	 			                                     */
/* Description:  implements basic control behavior for formation and         */
/*               flocking with e-puck 2 robots in webots                     */
/* Author: 	 Titouan Renard												     */
/*****************************************************************************/


#ifndef CONTOLLER_H
#define CONTOLLER_H

#define OBSTACLE_BUFFER_SIZE 100
#define MAX_OBSTACLE_AGE 100

#define MIGRATION_X 100
#define MIGRATION_Y 0

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include "../localization_controller/odometry.h"
#include "../communication/communication.h"


/* if defined, enables a second receiver on the epucks that listen for a PSO supervisor
    comment out for non supervised behavior */
// #define PSO

/* formation positions */

#define FORMATION_WIDTH 0.4

#if (ROBOT_NUMBER == 1)

#define SIN_PI_6 0.5
#define COS_PI_6 0.86602540378

static pose_t ref_poses[ROBOT_NUMBER] = {  {.x = 0, .y = 0, .heading = 0}};

#endif

#if (ROBOT_NUMBER == 4)

#define SIN_PI_6 0.5
#define COS_PI_6 0.86602540378

static pose_t ref_poses[ROBOT_NUMBER] = {  {.x = 0, .y = 0, .heading = 0},
                                    {.x = SIN_PI_6*FORMATION_WIDTH, .y = COS_PI_6*FORMATION_WIDTH, .heading = 0},
                                    {.x = SIN_PI_6*FORMATION_WIDTH, .y = -COS_PI_6*FORMATION_WIDTH, .heading = 0},
                                    {.x = -FORMATION_WIDTH, .y = 0, .heading = 0}};

#endif

#if (ROBOT_NUMBER == 5)

static pose_t ref_poses[ROBOT_NUMBER] = {  {.x = 0, .y = 0, .heading = 0},
                                    {.x = -FORMATION_WIDTH, .y = 0, .heading = 0},
                                    {.x = 0, .y = -FORMATION_WIDTH, .heading = 0},
                                    {.x = 0, .y = FORMATION_WIDTH, .heading = 0},
                                    {.x = FORMATION_WIDTH, .y = 0, .heading = 0}};

#endif

//wheel speed threshold
#define WS_THRESH 6.27

/* functions */

//initializes the proximity sensors
void init_prox_sensor();

// returns a migration urge vector (as a pose structure), passed by a pointer
void migration_urge(pose_t *migration, pose_t robot, pose_t goal);

// returns a consensus controller vector (as a pose structure), passed by a pointer
void consensus_controller(pose_t *consensus, pose_t robot_pose, pose_t *goal_pose, double kp, double ki, int robot_id, double* w);

// returns a reynold controller vector (as a pose structure), passed by a pointer
void reynolds_controller(pose_t *reynold, pose_t robot_pose, double w_cohesion, double w_dispersion, double w_consistency, int robot_id, double rule2radius);

// returns a local avoidance vector (as a pose structure), passed by a pointer
void unicycle_controller(double *omega, double *v, pose_t robot, pose_t goal, double ka, double kb, double kc);

// returns wheel speed from angular and linear velocity commands
void unicylce_to_wheels(double *w_left, double *w_right, double u_omega, double u_v, double l, double threshold);

void init_range_bearing_estimates(int* robot_id, pose_t* goal_pose);

void init_pso_reciever();

int get_hyperparameters_from_supervisor(double *hyperparameters);


#endif