#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "odometry.h"

//-----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.0201		// Radius of the wheel in meter

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC false     	// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC false    	// Print odometry values computed with accelerometer
#define VERBOSE_ODO_SPEED false		// Print the speed of the robot, in world frame, from the encoders
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static double integrated_speedxr, integrated_speedyr;
static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */
void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3], double Aleft_enc, double Aright_enc)
{
	
	double acc_yr = acc[0]-acc_mean[0];
	double acc_xr = acc[1]-acc_mean[1];
	
	
	//  Rad to meter : Convert the wheel encoders units into meters
	Aright_enc  =WHEEL_RADIUS*Aright_enc;
	Aleft_enc= WHEEL_RADIUS*Aleft_enc;
	// Comupute omega using the wheel encoders
	double omega = (Aright_enc-Aleft_enc)/WHEEL_AXIS/_T;
	
	double dxr, dyr, dthetar;
	double dspeedxr, dspeedyr;
	double integrated_speedxnr, integrated_speedynr;
	//compute the deltas in the robot frame
	if (fabs(omega) < 0.000001)
	{
		dxr = 0.5*acc_xr*_T*_T+integrated_speedxr*_T;
		dyr = 0.5*acc_yr*_T*_T+integrated_speedyr*_T;
		dspeedxr = acc_xr*_T;
		dspeedyr = acc_yr*_T;
		dthetar = 0;
	}
	else
	{
		dthetar = omega*_T;
		dxr =  acc_xr*(1-cos(dthetar))/omega/omega + acc_yr*(sin(dthetar))/omega/omega   - acc_yr*_T/omega+integrated_speedxr*_T;
		dyr = -acc_xr*(sin(dthetar))/omega/omega   + acc_yr*(1-cos(dthetar))/omega/omega + acc_xr*_T/omega+integrated_speedyr*_T;
		dspeedxr = acc_xr*sin(dthetar)/omega     - acc_yr*(1-cos(dthetar))/omega;
		dspeedyr = acc_xr*(1-cos(dthetar))/omega + acc_yr*sin(dthetar)/omega;
	}
	
	
	_odo_pose_acc.x += dxr*cos(_odo_pose_acc.heading)-dyr*sin(_odo_pose_acc.heading);
	_odo_pose_acc.y += dxr*sin(_odo_pose_acc.heading)+dyr*cos(_odo_pose_acc.heading);
	_odo_pose_acc.heading += dthetar;
	
	//integrated speed in the old robot frame
	integrated_speedxr += dspeedxr*cos(dthetar)-dspeedyr*sin(dthetar);
	integrated_speedyr += dspeedxr*sin(dthetar)+dspeedyr*cos(dthetar);
	
	//integrated speed in the new robot frame
	integrated_speedxnr = integrated_speedxr*cos(dthetar)-integrated_speedyr*sin(dthetar);
	integrated_speedynr = integrated_speedxr*sin(dthetar)+integrated_speedyr*cos(dthetar);
	
	//change the frame:
	integrated_speedxr = integrated_speedxnr;
	integrated_speedyr = integrated_speedynr;

	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
	
	if(VERBOSE_ODO_ACC)
    {
		printf("rawacc[0]: %f, rawacc[1]: %f, rawacc[2]: %f \n",acc[0],acc[1],acc[2]);
		//printf("accx: %f, accy: %f\n", acc_wx,acc_wy);
		//printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));
    }	
}

/**
 * @brief      	Compute the speed in x and y given a pose (for the heading)
 * 				and encoders measurements
 *
 * @param[out] speed     The derivative of the pose 
 * @param[in]  ref       The previous pose
 * @param[in]  Aleft_enc The value of the left encoder
 * @param[in]  Aright_enc The value of the right encoder
 */
void odo_compute_speed(pose_t* speed, const pose_t ref, double Aleft_enc, double Aright_enc)
{
	Aright_enc  =WHEEL_RADIUS*Aright_enc;

	Aleft_enc= WHEEL_RADIUS*Aleft_enc;

	// Comupute speeds : Compute the forward and the rotational speed
	
	double omega = (Aright_enc-Aleft_enc)/WHEEL_AXIS/_T;

	double vel =  (Aright_enc+Aleft_enc)/2/_T;

	
	//  Compute the speed into the world frame (A) 
	
	speed->x = cos(ref.heading)*vel;

	speed->y = sin(ref.heading)*vel;

	speed->heading  = omega;

	
}

/**
 * @brief      Compute the odometry using the encoders
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc)
{
	//  Rad to meter : Convert the wheel encoders units into meters
	
	Aright_enc  =WHEEL_RADIUS*Aright_enc;

	Aleft_enc= WHEEL_RADIUS*Aleft_enc;

	
	// Comupute speeds : Compute the forward and the rotational speed
	
	double omega = (Aright_enc-Aleft_enc)/WHEEL_AXIS/_T;

	double speed =  (Aright_enc+Aleft_enc)/2/_T;
	
	
	// Compute heading speed	
	double omega_w  = omega;
	// Use mid-way heading value for speed computation	
	double mid_way_heading = _odo_pose_enc.heading + 0.5*omega_w*_T;
	// Update heading
	_odo_pose_enc.heading += omega_w*_T;

	
	
	
	

	
	//  Compute the speed into the world frame (A) 
	
	double speed_wx = cos(mid_way_heading)*speed;

	double speed_wy = sin(mid_way_heading)*speed;



	// Integration : Euler method
	
	 _odo_pose_enc.x = _odo_pose_enc.x+speed_wx*_T;

	 _odo_pose_enc.y = _odo_pose_enc.y+speed_wy*_T;



	 memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

	 if(VERBOSE_ODO_ENC)
            	 printf("ODO with wheel encoders : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading) );
}


/**
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step)
{
	memset(&integrated_speedxr,0,sizeof(double));
	memset(&integrated_speedyr,0,sizeof(double));

 	memset(&_odo_pose_acc, 0 , sizeof(pose_t));

	memset(&_odo_speed_acc, 0 , sizeof(pose_t));

	memset(&_odo_pose_enc, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
}



pose_t pose_scale(double c, pose_t p){
	pose_t new_pose;
	new_pose.x = c * p.x;
	new_pose.y = c * p.y;
	new_pose.heading = c * p.heading;
	return new_pose;
}

pose_t pose_add(pose_t a, pose_t b){
	pose_t new_pose;
	new_pose.x = a.x + b.x;
	new_pose.y = a.y + b.y;
	new_pose.heading = a.heading + b.heading;
	return new_pose;
}

pose_t pose_add_3(pose_t a, pose_t b, pose_t c){
	pose_t new_pose;
	new_pose.x = a.x + b.x + c.x;
	new_pose.y = a.y + b.y + c.y;
	new_pose.heading = a.heading + b.heading + c.heading;
	return new_pose;
}