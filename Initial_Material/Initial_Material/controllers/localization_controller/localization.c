
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
#define VERBOSE_KALMAN false
/*DEFINITIONS*/


/*VARIABLES*/
static sensors_t   		_sensors;
static measurement_t 	_meas;
static control_t 		_control;
static pose_t        	_pose_gps, _pose_acc, _pose_enc, _pose_kalman;
static pose_t        	_pose_origin = {-0.0, 0.0, 0.0};
static double 			last_gps_time_s = -1.1f;
static int 				_time_step;
static FILE *fp;

/* FUNCTIONS */
static bool loc_init_encoder(int time_step);
static bool loc_init_acc(int time_step);
static bool loc_init_gps();
static bool loc_init_kalman(int ts);

static bool loc_init_log(const char* filename);

static void loc_compute_mean_acc(int time_init, int time_step);

static void loc_get_encoder();
static void loc_get_acc();
static void loc_get_gps();

static void loc_get_gps_pose();
static double loc_get_heading();
void loc_recompute_B(double B[N_STATES][N_CONTROL_INPUT],double theta);
static void loc_format_u(double u[N_CONTROL_INPUT]);
static void loc_format_y(double y_meas[N_OBSERVABLES], int freq);
static void loc_format_kalman_pose();

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
	_time_step = time_step;
	bool err = false;
	
	memset(&_sensors, 0 , sizeof(sensors_t));

	memset(&_meas, 0 , sizeof(measurement_t));

	memset(&_pose_gps, 0, sizeof(pose_t));

	memset(&_pose_enc, 0 , sizeof(pose_t));

	memset(&_pose_acc, 0 , sizeof(pose_t));
	
	memset(&_pose_kalman, 0 , sizeof(pose_t));
	switch (MODE_LOC)
	{
		case ODOM:
			CATCH(err,loc_init_encoder(time_step));
			odo_reset(time_step);
			break;
		case ODOM_ACC:
			CATCH(err,loc_init_encoder(time_step));
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
			CATCH(err,loc_init_kalman(time_step));
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

bool loc_init_kalman(int ts)
{
	double state_0[N_STATES];
	double cov_0[N_STATES][N_STATES];
	double A[N_STATES][N_STATES];
	double B[N_STATES][N_CONTROL_INPUT];
	double C[N_FREQ][N_OBSERVABLES][N_STATES];
	double D[N_FREQ][N_OBSERVABLES][N_CONTROL_INPUT];
	double Q[N_STATES][N_STATES];
	double R[N_FREQ][N_OBSERVABLES][N_OBSERVABLES];
	
	double ts_mil = ts/1000.;
	
	
	if(MODE_KAL == ACC_CONTROLLED)
	{
		for(int i = 0; i<N_STATES;i++)
		{
			
			state_0[i] = 0;
			for(int j = 0;j<N_STATES;j++)
			{
				if(i==j)
					cov_0[i][j] = 0.0001;
				else
					cov_0[i][j] = 0;
			}
		}
		//Init A
		memset(A,0,sizeof(A));
		A[0][0] = 1;
		A[0][2] = ts_mil;
		A[1][1] = 1;
		A[1][3] = ts_mil;
		A[2][2] = 1;
		A[3][3] = 1;
		//Init B
		B[0][0] = ts_mil*ts_mil/2;
		B[0][1] = 0;
		B[1][0] = 0;
		B[1][1] = ts_mil*ts_mil/2;
		B[2][0] = ts_mil;
		B[2][1] = 0;
		B[3][0] = 0;
		B[3][1] = ts_mil;
		
		//Init C
		memset(C,0,sizeof(C));
		C[F_ODOM][0][2] = 1;
		C[F_ODOM][1][3] = 1;
		C[F_GPS][0][0] = 1;
		C[F_GPS][1][1] = 1;
		   

		//Init D
		memset(D,0,sizeof(D));
		

		//Init Q 
		memset(Q,0,sizeof(Q));
		Q[0][0] = 0.1*ts_mil/1000.;
		Q[1][1] = 0.1*ts_mil/1000.;
		Q[2][2] = 0.1*ts_mil/1000.;
		Q[3][3] = 0.1*ts_mil/1000.;

		//Init R
		memset(R,0,sizeof(R));
		R[F_ODOM][0][0] = 0.05*ts_mil/1000.; //proportional to ts
		R[F_ODOM][1][1] = 0.05*ts_mil/1000.;
		R[F_GPS][0][0] = 0.0001/1000.;		 //resolution of the GPS
		R[F_GPS][1][1] = 0.0001/1000.;
				   
		kal_init_kalman(state_0,cov_0,A,B,Q,C,D,R);
	}
	if(MODE_KAL == ACC_CONTROLLED_UPG)
	{
		for(int i = 0; i<N_STATES;i++)
		{
			
			state_0[i] = 0;
			for(int j = 0;j<N_STATES;j++)
			{
				if(i==j)
					cov_0[i][j] = 0.0001;
				else
					cov_0[i][j] = 0;
			}
		}
		//Init A
		memset(A,0,sizeof(A));
		A[0][0] = 1;
		A[0][3] = ts_mil;
		A[1][1] = 1;
		A[1][4] = ts_mil;
		A[2][2] = 1;
		A[2][5] = ts_mil;
		A[3][3] = 1;
		A[4][4] = 1;
		A[5][5] = 1;
		
		//Init B
		memset(B,0,sizeof(B));
		B[0][0] = ts_mil*ts_mil/2;
		B[1][1] = ts_mil*ts_mil/2;
		B[3][0] = ts_mil;
		B[4][1] = ts_mil;
		
		//Init C
		memset(C,0,sizeof(C));
		C[F_ODOM][0][3] = 1;
		C[F_ODOM][1][4] = 1;
		C[F_ODOM][2][5] = 1;
		C[F_GPS][0][0] = 1;
		C[F_GPS][1][1] = 1;
		   

		//Init D
		memset(D,0,sizeof(D));
		

		//Init Q 
		memset(Q,0,sizeof(Q));
		Q[0][0] = 0.1*ts_mil/1000.;
		Q[1][1] = 0.1*ts_mil/1000.;
		Q[2][2] = 2*M_PI; //we actually don't have any information on theta for now
		Q[3][3] = 0.1*ts_mil/1000.;
		Q[4][4] = 0.1*ts_mil/1000.;
		Q[5][5] = 2*M_PI; //we actually don't have any information on theta for now
		
		//Init R
		memset(R,0,sizeof(R));
		R[F_ODOM][0][0] = 0.05*ts_mil/1000.;
		R[F_ODOM][1][1] = 0.05*ts_mil/1000.;
		R[F_ODOM][2][2] = 1.75*ts_mil/1000.; //considering that each wheel is subjected to a random noise, then
											 //sigma angular = sigma straight *2/WHEEL_AXIS
		R[F_GPS][0][0] = 0.0001*ts_mil/1000.;
		R[F_GPS][1][1] = 0.0001*ts_mil/1000.;
		R[F_GPS][2][2] = 1;					//needed for mathematical stability
		kal_init_kalman(state_0,cov_0,A,B,Q,C,D,R);
	}
	
	return false;
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
    fprintf(fp, "time; pose_x; pose_y; pose_heading;  gps_x; gps_y; gps_z; acc_0; acc_1; acc_2; right_enc; left_enc; odo_acc_x; odo_acc_y; odo_acc_heading; odo_enc_x; odo_enc_y; odo_enc_heading; kalman_x; kalman_y; kalman_heading; mode_x; mode_y; mode_heading \n");
  }

  return err;
}

//calibration functions
/** 
 * @brief	Achieve the calibration: if the MODE_LOC is either ODOM_ACC or KALMAN,
 * 			compute the mean acceleration to remove the bias
 */
void loc_calibrate(int time_init, int time_step)
{
	switch(MODE_LOC)
	{
		case ODOM_ACC:
		case KALMAN:
			loc_update_measures();
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
			_meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 21) + _meas.acc[i]) / ((double) count-20);
	}

	if( count == (int) (time_init / (double) time_step * 1000) )
		printf("Accelerometer initialization Done ! \n");

	if(VERBOSE_ACC_MEAN)
		printf("ROBOT acc mean : %g %g %g\n", _meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2]);
}

//Measure functions
/**
 * @brief	get the measurments required in MODE_LOC from webots
 */
void loc_update_measures()
{
	switch (MODE_LOC)
	{
		case ODOM:
			loc_get_encoder();
			break;
		case ODOM_ACC:
			loc_get_encoder();
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
			printf("unknown MODE_LOC\n");
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
	{
		printf("ROBOT acc : %g %g %g\n", _meas.acc[0], _meas.acc[1] , _meas.acc[2]);
	}
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
 * @brief		Compute the pose required in the selected MODE_LOC
 */
void loc_compute_pose()
{
	double u[N_CONTROL_INPUT];
	double y_meas[N_OBSERVABLES];
	double B[N_STATES][N_CONTROL_INPUT];
	switch(MODE_LOC)
	{
		case ODOM:
			odo_compute_encoders(&_pose_enc, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
			break;
			
		case ODOM_ACC:
			//needed to find the heading of the robot
			odo_compute_acc(&_pose_acc, _meas.acc, _meas.acc_mean, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
			
			break;
		case GPS:
			loc_get_gps_pose();
			break;
		case KALMAN:
			loc_get_gps_pose();
			if(MODE_KAL == ACC_CONTROLLED)
				odo_compute_acc(&_pose_acc, _meas.acc, _meas.acc_mean, _meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
			loc_format_y(y_meas,F_ODOM);
			kal_update_freq(F_ODOM,y_meas);
			
			

			
			
			if(fabs(last_gps_time_s-wb_robot_get_time())<TOL)
			{
				if VERBOSE_KALMAN
					printf("UPDATE GPS\n");
				loc_format_y(y_meas,F_GPS);
				kal_update_freq(F_GPS,y_meas);
			}
			
			//the retrurned pose will be the last one measured
			loc_format_kalman_pose();
			
			//predict the future pose
			if(MODE_KAL == ACC_CONTROLLED_UPG || MODE_KAL == ACC_CONTROLLED)
			{
				loc_format_u(u);
				kal_predict(u);
			}
			
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
      printf("ROBOT pose GPS : %g %g %g\n", _pose_gps.x , _pose_gps.y , RAD2DEG(_pose_gps.heading));
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

//Kalman formating

void loc_format_u(double u[N_CONTROL_INPUT]){
	//compute the part caused by the control
	if(MODE_KAL == ACC_CONTROLLED || MODE_KAL == ACC_CONTROLLED_UPG)
	{
		//printf("heading: %f \n",_pose_acc.heading);
		u[0]  =  cos(_pose_kalman.heading)*(_meas.acc[1]-_meas.acc_mean[1])
				+sin(_pose_kalman.heading)*(_meas.acc[0]-_meas.acc_mean[0]);
		u[1] = 	-sin(_pose_kalman.heading)*(_meas.acc[1]-_meas.acc_mean[1])
				-cos(_pose_kalman.heading)*(_meas.acc[0]-_meas.acc_mean[0]);
	}
}

void loc_format_y(double y_meas[N_OBSERVABLES], int freq)
{
	if(MODE_KAL == ACC_CONTROLLED)
	{
		if(freq == F_GPS)
		{
			y_meas[0] = _pose_gps.x;
			y_meas[1] = _pose_gps.y;		
			//all other possible observables are set to 0 (note that they would have a K[i] 
			//equal to 0 anyways)
			for(int i = 2;i<N_OBSERVABLES;i++)
				y_meas[i] = 0;
		}
		if(freq == F_ODOM)
		{
			pose_t speed;
			
			odo_compute_speed(&speed,_pose_kalman,_meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
			
			y_meas[0] = speed.x;
			y_meas[1] = speed.y;
		}
	}
	if(MODE_KAL == ACC_CONTROLLED_UPG)
	{
		if(freq == F_GPS)
		{
			y_meas[0] = _pose_gps.x;
			y_meas[1] = _pose_gps.y;
			y_meas[2] = _pose_gps.heading;			
			//all other possible observables are set to 0 (note that they would have a K[i] 
			//equal to 0 anyways)
			for(int i = 3;i<N_OBSERVABLES;i++)
				y_meas[i] = 0;
		}
		if(freq == F_ODOM)
		{
			pose_t speed;
			
			odo_compute_speed(&speed,_pose_kalman,_meas.left_enc - _meas.prev_left_enc, _meas.right_enc - _meas.prev_right_enc);
			
			y_meas[0] = speed.x;
			y_meas[1] = speed.y;
			y_meas[2] = speed.heading;
		}
	}

}
void loc_format_kalman_pose()
{
	if(MODE_KAL	== ACC_CONTROLLED)
	{
		double x[N_STATES];
		kal_get_pose(x);
		_pose_kalman.x = x[0];
		_pose_kalman.y = x[1];
		_pose_kalman.heading=_pose_acc.heading;
	}
	if(MODE_KAL == ACC_CONTROLLED_UPG)
	{
		double x[N_STATES];
		kal_get_pose(x);
		_pose_kalman.x = x[0];
		_pose_kalman.y = x[1];
		_pose_kalman.heading=x[2];
	}
	if(VERBOSE_KALMAN)
		printf("ROBOT pose : %g %g %g\n", _pose_kalman.x , _pose_kalman.y , RAD2DEG(_pose_kalman.heading));
}

// Output functions
pose_t loc_get_pose()
{
	switch(MODE_LOC)
	{
		case ODOM:
			return _pose_enc;
		case ODOM_ACC:
			return _pose_acc;
		case GPS:
			return _pose_gps;
		case KALMAN:
		default:
			return _pose_kalman;
		
	}
}
//LOG function
void loc_print_log(double time)
{
	if(LOGS){
		if( fp != NULL)
		{
			pose_t pose_mode = loc_get_pose();
			fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g;  %g; %g; %g; %g; %g; %g\n",
					time, _pose_gps.x, _pose_gps.y , _pose_gps.heading, _meas.gps[0], _meas.gps[1], 
					_meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
					_pose_acc.x, _pose_acc.y, _pose_acc.heading, _pose_enc.x, _pose_enc.y, _pose_enc.heading, 
					_pose_kalman.x, _pose_kalman.y, _pose_kalman.heading, pose_mode.x, pose_mode.y, pose_mode.heading);
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

    fprintf(stderr,"%s",buffer);

    return(true);
  }

  return false;
}

