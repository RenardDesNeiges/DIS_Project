#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H 

#include <webots/robot.h>
#include <webots/motor.h>

// ## DO NOT MODIFY THIS
void trajectory_1(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor);
void trajectory_2(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor);

// ## but you can add your own trajectories if you like.
void trajectory_1_delay(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor,int init_time);
void circle_delay(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor, int init_time);
void lin_delay(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor, int init_time);
void turn_over_itself_delay(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor, int init_time);
#endif
