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
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */
void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3])
{
	// To Do : Compute the acceleration in frame A + remove biais (Assume 1-D motion)
	double acc_wx = acc[1]-acc_mean[1];

	// To Do : Implement your motion model (Assume 1-D motion)
	
	_odo_pose_acc.x += _odo_speed_acc.x *_T+0.5*acc_wx*_T*_T;
	
	_odo_speed_acc.x +=acc_wx*_T;

	

	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
	
	if(VERBOSE_ODO_ACC)
    {
		printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));
    }	
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
	double mid_way_heading = 0.5*_odo_pose_enc.heading + omega_w*_T;
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

 	memset(&_odo_pose_acc, 0 , sizeof(pose_t));

	memset(&_odo_speed_acc, 0 , sizeof(pose_t));

	memset(&_odo_pose_enc, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
}
