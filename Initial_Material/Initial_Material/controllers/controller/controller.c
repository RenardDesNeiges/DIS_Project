#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <math.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include "../localization_controller/odometry.h"

#include "controller.h"

void migration_urge(pose_t *migration, pose_t robot, pose_t goal)
{
    double dist = pow( pow(goal.x-robot.x,2) + pow(goal.y-robot.y,2), 0.5);
    if(dist > 1)
    {
        migration->x = (goal.x-robot.x)/dist;
        migration->y = (goal.y-robot.y)/dist;
    }
    else
    {
        migration->x = (goal.x-robot.x);
        migration->y = (goal.y-robot.y);   
    }
    migration->heading = goal.heading;
}

void concensus_controller(pose_t *concensus, pose_t robot)
{

}

void local_avoidance_controller(pose_t *local, pose_t robot)
{

}

void unicylce_controller(double *omega, double *v, pose_t robot, pose_t goal, double ka, double kb, double kc)
{   

    double beta = atan2(goal.y, goal.x);
    double alpha = robot.heading - beta;
    double theta = goal.heading;
    double e = pow( pow(goal.x,2) + pow(goal.y,2), 0.5);
    // printf("%f %f,%f \n", e, beta, alpha);
    if(fabs(e) < 0.006) //avoids oscillations
        e = 0;
    if(fabs(alpha) < 0.2) //avoids oscillations
        alpha = 0;
    *v = (ka * cos(alpha)) * e;
    if(alpha != 0)
        *omega = kb * alpha + ka * (cos(alpha)*sin(alpha))/alpha * (alpha + kc * theta);
    else // when alpha tends to zero the cos*sin/alpha term tends to 1, but c doesn't know that ...
        *omega = kb * alpha + ka * (alpha + kc * theta);
    
    printf("e = %f, alpha = %f, v = %f, omega = %f \n", e, alpha, *v, *omega);
}

void unicylce_to_wheels(double *w_left, double *w_right, double u_omega, double u_v, double l)
{   
    double threshold = 6.275; //max value minus a small epsilon
    double normalize = 1;
    double left = u_v + (l/2) * u_omega;
    double right = u_v - (l/2) * u_omega;
    if(fabs(left) > threshold)
        normalize = threshold/fabs(left);
    if (fabs(right) > threshold)
        if(fabs(right) > fabs(left))
            normalize = threshold/fabs(right);
    *w_left = left*normalize;
    *w_right = right*normalize;
    printf("left = %f, right = %f \n",*w_left,*w_right);
}

