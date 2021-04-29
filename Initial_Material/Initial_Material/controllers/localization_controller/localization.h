#ifndef LOCALIZATION_H
#define LOCALIZATION_H
//Measurement method
#define GPS 0
#define ODOM 1
#define ODOM_ACC 2
#define KALMAN 3

#define MODE KALMAN
#define LOGS	1 		//set to 1 to write logs in DATANAME
#define DATANAME "localization.csv"
//-----------------------------------------------------------------------------------//
/*DEFINITIONS*/
typedef struct
{
  int time_step;
  WbDeviceTag gps;
  WbDeviceTag acc;
  WbDeviceTag left_encoder;
  WbDeviceTag right_encoder;
} sensors_t;


bool loc_init(int time_step, pose_t pose_origine);

void loc_calibrate(int time_init, int time_step);

void loc_update_measures();

void loc_compute_pose();

pose_t get_pose();

void loc_print_log(double time);



#endif

