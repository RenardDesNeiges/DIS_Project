
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>


#include "odometry.h"
#include "kalman.h"
#include "localization.h"
//-----------------------------------------------------------------------------------//
/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  loc_error(X, Y, __LINE__, __FILE__)

/*CONSTANTES*/
#define TOL 0.001

/*VERBOSE_FLAGS*/
#define VERBOSE_GPS false        // Print GPS values
#define VERBOSE_ACC false       // Print accelerometer values
#define VERBOSE_ACC_MEAN false  // Print accelerometer mean values
#define VERBOSE_POSE_GPS false      // Print pose values
#define VERBOSE_ENC false       // Print encoder values

/*DEFINITIONS*/


/*VARIABLES*/
static sensors_t   		_sensors;
static measurement_t 	_meas;
static pose_t        	_pose_gps, _pose_acc, _pose_enc, _pose_kalman;
static pose_t        	_pose_origin = {-0.0, 0.0, 0.0};
double last_gps_time_s = 1.1f;
static FILE *fp;

/* FUNCTIONS */
static bool loc_init_encoder(int time_step);
static bool loc_init_acc(int time_step);
static bool loc_init_gps();
static bool loc_init_log(const char* filename);

static void loc_compute_mean_acc(int time_init, int time_step);

static void loc_get_encoder();
static void loc_get_acc();
static void loc_get_gps();

static void loc_get_gps_pose();
static double loc_get_heading();

static bool loc_error(bool test,const char * message, int line, const char * fileName);
//-----------------------------------------------------------------------------------//

// --------------------- INITIALISATION --------------- //
/**
 * @brief      Initialize the necessary sensors from Webots (given MODE)
 *
 * @return     return true if it fails
 */
bool loc_init(int time_step, pose_t pose_origine) 
{
	_pose_origin = pose_origine;
	
	bool err = false;

	memset(&_sensors, 0 , sizeof(sensors_t));

	memset(&_meas, 0 , sizeof(measurement_t));

	memset(&_pose_gps, 0, sizeof(pose_t));

	memset(&_pose_enc, 0 , sizeof(pose_t));

	memset(&_pose_acc, 0 , sizeof(pose_t));
	
	memset(&_pose_kalman, 0 , sizeof(pose_t));
	switch (MODE)
	{
		case ODOM:
			CATCH(err,loc_init_encoder(time_step));
			odo_reset(time_step);
			break;
		case ODOM_ACC:
			CATCH(err,loc_init_acc(time_step));
			odo_reset(time_step);
			break;
		case GPS:
			CATCH(err,loc_init_gps());
			break;
		case KALMAN:
			CATCH(err,loc_init_encoder(time_step));
			CATCH(err,loc_init_acc(time_step));
			CATCH(err,loc_init_gps());
			odo_reset(time_step);
			break;
		default:
			printf( "loc fails to init: mode unknown \n");
			return true;
			break;


		
  
	}
	if (LOGS)
	{
		CATCH(err, loc_init_log(DATANAME));
	}
	return err;
}
//INIT functions
/**
 * @brief      Initialize the gps sensor from Webots
 *
 * @return     return true if it fails
 */
bool loc_init_gps() 
{
	// Get the gps sensor from webots.
	_sensors.gps = wb_robot_get_device("gps");

	bool err = CATCH_ERR(_sensors.gps == 0, "No gps node found in the current robot file\n");

	if( !err )
	{
		// Enable the sensor on Webots.  (note that it is updated every 1000ms)
		wb_gps_enable(_sensors.gps, 1000);    
	}
	//initialize the data
	loc_get_gps();

	return err;
}
/**
 * @brief       Initialize the accelerometer sensor from Webots
 *
 * @return      return true if it fails
 */
bool loc_init_acc(int time_step)
{
//Get the acc sensor from webots. 
  _sensors.acc = wb_robot_get_device("accelerometer");

  bool err = CATCH_ERR(_sensors.acc == 0, "No accelerometer node found in the current robot file\n");

  if( !err )
  {
    // Enable the sensor on Webots 
    wb_accelerometer_enable(_sensors.acc, time_step);
  }

  return err;
}
/**
 * @brief      Initiliaze the the wheel encoders (position sensors) from Webots
 *
 * @return      return true if it fails
 */
bool loc_init_encoder(int time_step)
{
  // Get the device from Webots
  _sensors.left_encoder = wb_robot_get_device("left wheel sensor");
  // Write an error message if the initialization fails
  bool err = CATCH_ERR(_sensors.left_encoder == 0, "No left wheel sensor node found in the current robot file\n");

  _sensors.right_encoder = wb_robot_get_device("right wheel sensor");

  CATCH(err,CATCH_ERR(_sensors.right_encoder == 0, "No right wheel sensor node found in the current robot file\n"));
  
  if( !err ) // if no error initialize the sensors
  {
    wb_position_sensor_enable(_sensors.left_encoder,  time_step);
    wb_position_sensor_enable(_sensors.right_encoder, time_step);
  }

  return err;
}
/**
 * @brief      Initialize the logging of the file
 *
 * @param[in]  filename  The filename to write
 *
 * @return     return true if it fails
 */
bool loc_init_log(const char* filename)
{

  fp = fopen(DATANAME,"w");
  
  bool err = CATCH_ERR(fp == NULL, "Fails to create a log file\n");

  if( !err )
  {
    fprintf(fp, "time; pose_x; pose_y; pose_heading;  gps_x; gps_y; gps_z; acc_0; acc_1; acc_2; right_enc; left_enc; odo_acc_x; odo_acc_y; odo_acc_heading; odo_enc_x; odo_enc_y; odo_enc_heading; kalman_x; kalman_y; kalman_heading\n");
  }

  return err;
}

//calibration functions
/** 
 * @brief	Achieve the calibration: if the mode is either ODOM_ACC or KALMAN,
 * 			compute the mean acceleration to remove the bias
 */
void loc_calibrate(int time_init, int time_step)
{
	switch(MODE)
	{
		case ODOM_ACC:
		case KALMAN:
			loc_compute_mean_acc(time_init,time_step);
			break;
	}
}
/**
 * @brief      Compute the mean of the 3-axis accelerometer. The result is stored in array _meas.acc
 */
static void loc_compute_mean_acc(int time_init,int time_step)
{

	static int count = 0;

	count++;

	if( count > 20 ) // Remove the effects of strong acceleration at the begining
	{
		for(int i = 0; i < 3; i++)  
			_meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 1) + _meas.acc[i]) / (double) count;
	}

	if( count == (int) (time_init / (double) time_step * 1000) )
		printf("Accelerometer initialization Done ! \n");

	if(VERBOSE_ACC_MEAN)
		printf("ROBOT acc mean : %g %g %g\n", _meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2]);
}

//Measure functions
/**
 * @brief	get the measurments required in MODE from webots
 */
void loc_update_measures()
{
	switch (MODE)
	{
		case ODOM:
			loc_get_encoder();
			break;
		case ODOM_ACC:
			loc_get_acc();
			break;
		case GPS:
			loc_get_gps();
			break;
		case KALMAN:
			loc_get_encoder();
			loc_get_acc();
			loc_get_gps();
			break;
		default:
			printf("unknown mode\n");
			break;
	}
}
/**
 * @brief      Read the encoders values from the sensors
 */
void loc_get_encoder()
{
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(_sensors.left_encoder);
  
  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;
  
  _meas.right_enc = wb_position_sensor_get_value(_sensors.right_encoder);

  if(VERBOSE_ENC)
    printf("ROBOT enc : %g %g\n", _meas.left_enc, _meas.right_enc);
}
/**
 * @brief      Read the acceleration values from the sensor
 */
void loc_get_acc()
{
  const double * acc_values = wb_accelerometer_get_values(_sensors.acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  if(VERBOSE_ACC)
    printf("ROBOT acc : %g %g %g\n", _meas.acc[0], _meas.acc[1] , _meas.acc[2]);
}
/**
 * @brief   	Get the gps measurments for position of the robot if the 
 * 				last time it was sent was more than 1s ago
 */
void loc_get_gps()
{
	double time_now_s = wb_robot_get_time();
	
	//check if the  GPS sent a new position
	if (time_now_s - last_gps_time_s > 1.0f)
	{
		const double * gps_position = wb_gps_get_values(_sensors.gps);
		last_gps_time_s = time_now_s;
		
		memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
		memcpy(_meas.gps, gps_position, sizeof(_meas.gps));

		if(VERBOSE_GPS)
			printf("ROBOT gps is at position: %g %g %g\n", _meas.gps[0], _meas.gps[1], _meas.gps[2]);
	}
}

// Compute the poses
/** 
 * @brief		Compute the pose required in the selected MODE
 */
void loc_compute_pose()
{
	switch(MODE)
	{
		case ODOM:
			odo_compute_encoders(&_pose_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
			break;
		case ODOM_ACC:
			odo_compute_acc(&_pose_acc, _meas.acc, _meas.acc_mean);
			break;
		case GPS:
			loc_get_gps_pose();
			break;
		case KALMAN:
			odo_compute_acc(&_pose_acc, _meas.acc, _meas.acc_mean);
			loc_get_gps_pose();
			odo_compute_encoders(&_pose_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
			break;
		default: 
			break;
	}
}
/**
 * @brief     Get the gps measurements for the position of the robot. Get the heading angle. Fill the pose structure. 
 */
void loc_get_gps_pose()
{

    _pose_gps.x = _meas.gps[0] - _pose_origin.x;
    
    _pose_gps.y = -(_meas.gps[2] - _pose_origin.y);
    
    _pose_gps.heading = loc_get_heading() + _pose_origin.heading;
  
    if(VERBOSE_POSE_GPS)
      printf("ROBOT pose : %g %g %g\n", _pose_gps.x , _pose_gps.y , RAD2DEG(_pose_gps.heading));
}
/**
 * @brief      Compute the heading (orientation) of the robot based on the gps position values.
 *
 * @return     return the computed angle
 */
double loc_get_heading()
{
  double delta_x = _meas.gps[0] - _meas.prev_gps[0];

  double delta_y = -(_meas.gps[2] - _meas.prev_gps[2]);
  
  double heading = atan2(delta_y, delta_x);
  
  return heading;
}

// Output functions
pose_t get_pose()
{
	switch(MODE)
	{
		case ODOM:
			return _pose_enc;
		case ODOM_ACC:
			return _pose_acc;
		case GPS:
			return _pose_gps;
		case KALMAN:
			return _pose_kalman;
		default:
			//should never be reached, but if so let's just say it's a kalman
			return _pose_kalman;
	}
}
//LOG function
void loc_print_log(double time)
{
	if(LOGS){
		if( fp != NULL)
		{
			fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g;  %g; %g; %g;\n",
					time, _pose_gps.x, _pose_gps.y , _pose_gps.heading, _meas.gps[0], _meas.gps[1], 
					_meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
					_pose_acc.x, _pose_acc.y, _pose_acc.heading, _pose_enc.x, _pose_enc.y, _pose_enc.heading, 
					_pose_kalman.x, _pose_kalman.y, _pose_kalman.heading);
		}
	}

}

// -----------ERROR FUNCTION ----------


/**
 * @brief      Do an error test if the result is true write the message in the stderr.
 *
 * @param[in]  test     The error test to run
 * @param[in]  message  The error message
 *
 * @return     true if there is an error
 */
bool loc_error(bool test, const char * message, int line, const char * fileName)
{
  if (test) 
  {
    char buffer[256];

    sprintf(buffer, "file : %s, line : %d,  error : %s", fileName, line, message);

    fprintf(stderr,buffer);

    return(true);
  }

  return false;
}

