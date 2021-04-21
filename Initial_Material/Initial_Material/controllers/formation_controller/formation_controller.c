//******************************************************************************
//  Name:   obstacles.c
//  Author: Titouan Renard
//  Date:   March, 11th, 2021
//******************************************************************************

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <webots/motor.h>

#define NB_SENSORS 8
#define TIME_STEP 64
#define MAX_SPEED 1000
#define MAX_SENS 4095
#define MAX_SPEED_WEB      6.28    // Maximum speed webots

WbDeviceTag ps[NB_SENSORS]; // list of distance sensor handles
// note: the handles are != the sensor values. Sensor values
// are saved in ds_value[NB_SENSORS], which is defined in the main loop
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot

//-----------------------------------------------fitness evaluation-------------
// The fitness evaluation functions are provided to help you testing the
// performance of your controller.

// fitness variables
int step_count;
double fit_speed;
double fit_diff;
double sens_val[NB_SENSORS];

struct vector {
  float x;
  float y;
};

struct state {
  struct vector p;
  struct vector v;
};

//------------------------------------------------------------------------------

// controller initialization
static void reset(void) {
  int i;
  char name[] = "ps0";
  for(i = 0; i < NB_SENSORS; i++) {
    ps[i]=wb_robot_get_device(name); // get sensor handle
    // perform distance measurements every TIME_STEP millisecond
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    name[2]++; // increase the device name to "ps1", "ps2", etc.
  }
  
  //get motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
}

// controller main loop
static int run(int ms) {
  static double ds_value[NB_SENSORS]; //distance values for each sensor
  int i;
  float msl_w, msr_w;
    
  for (i = 0; i < NB_SENSORS; i++)
    // read sensor values
    // reads the handle in ps[i] and saves its value in ds_value[i]
    ds_value[i] = wb_distance_sensor_get_value(ps[i]); // range: 0 (far) to 4095 (0 distance (in theory))

  // choose behavior
  double left_speed, right_speed;
  int duration;
  //printf("ds_value[0] = %f\n", ds_value[0]);

  /* 
    General Idea behind the reactive controller : two layers,
    - control layer tries to follow a target vector
    - sensing layer generates a target vector from sensor data
  */

  // target vector computation : 
  int sense = 1;
  float vx = 0;
  float vy = 0;
  float dx = 0;
  float dy = 0;
  float sd = 0;
  float sx[8] = {0.2588,0.7071,1, 0.5736,-0.5736,-1,-0.7071,-0.2588};
  float sy[8] = {0.9659,0.7071,0,-0.8192,-0.8192, 0, 0.7071, 0.9659};
  for (i = 0; i < NB_SENSORS; i++)
    if(ds_value[i] > 300)
      sense = sense * 0;
  
  if(sense == 1){ // we have no obstacle, so go forward
    vy = 1;
    vx = 0;
  }
  else{ // we have an obstacle, compute a target vector
    for (i = 0; i < NB_SENSORS; i++){
      if(ds_value[i] > 300){
        dx = dx + (ds_value[i]/1000)*sx[i];
        dy = dy + (ds_value[i]/1000)*sy[i];
        sd = sd + sx[i]*sx[i] + sy[i]*sy[i];
      }
    }
    vx = 0.0 - dx/sqrt(sd); // (negative because we want to avoid the obstacle)
    vy = 0.3 - dy/sqrt(sd); // (negative because we want to avoid the obstacle)
  }
  
  printf("vx = %f ; vy = %f ; \n",vx,vy);

  float fwdspeed;
  if(vy > 0.3)
    fwdspeed = 700 * vy;
  else
    fwdspeed = 0;

  if(vy < 0.3 && (vx*vx) < 0.05){ // if we have an obstacle right in front of us, rotate the vehicle aggressively (to prevent oscillations when facing an obstacle)
    left_speed = 400.0;
    right_speed = -400.0;
    duration = 3 * ms; // for 1280 milliseconds
  }
  else{ //else compute trajectory from the velocity vector vx
    left_speed =  fwdspeed + 2500*vx;
    right_speed = fwdspeed - 2500*vx;
  duration = ms; // for 64 milliseconds
  }
  printf("ls = %f ; rs = %f ; \n",left_speed,right_speed);

  // if (ds_value[0] > 512) { // we detected an obstacle with IR0
  //   left_speed = -400;
  //   right_speed = 400; // turn around
  //   duration = 20 * ms; // for 1280 milliseconds
  // }
  // else {
  //   left_speed = right_speed = 500; // go straight (changing this speed can directly influence fitness)
  //   duration = ms; // for 64 milliseconds
  // }

  // actuate wheel motors
  // sets the e-pucks wheel speeds to left_speed (left wheel) and right_speed (right wheel)
  // max speed is 1000 == 2 turns per second
  // Set speed
  msl_w = left_speed*MAX_SPEED_WEB/1000;
  msr_w = right_speed*MAX_SPEED_WEB/1000;
  wb_motor_set_velocity(left_motor, msl_w);
  wb_motor_set_velocity(right_motor, msr_w);

  return duration;
}

int main() {
  int duration = TIME_STEP;

  wb_robot_init(); // controller initialization
  reset();
  
  
  while (wb_robot_step(duration) != -1) {
    duration = run(TIME_STEP);
  }
  
  wb_robot_cleanup();
  return 0;
}
