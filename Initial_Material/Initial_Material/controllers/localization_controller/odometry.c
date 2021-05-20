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

static double integrated_speedxw, integrated_speedyw;
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
	
	double acc_ry = acc[0]-acc_mean[0];
	double acc_rx = acc[1]-acc_mean[1];
	
	_odo_pose_acc.heading = _odo_pose_enc.heading;
	
	double acc_wx = cos(_odo_pose_acc.heading)*acc_rx+sin(_odo_pose_acc.heading)*acc_ry;
	double acc_wy = -sin(_odo_pose_acc.heading)*acc_rx-cos(_odo_pose_acc.heading)*acc_ry;
	double dxw,dyw;
	//kind of cheating that allows to bypass the absence of gyroscope
	_odo_pose_acc.heading = _odo_pose_enc.heading;
	
	dxw = integrated_speedxw*_T+0.5*acc_wx*_T*_T;
	dyw = integrated_speedyw*_T+0.5*acc_wy*_T*_T;
	_odo_pose_acc.x += dxw;
	_odo_pose_acc.y += dyw;

	integrated_speedxw +=acc_wx*_T;
	integrated_speedyw +=acc_wy*_T;
	
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
	memset(&integrated_speedxw,0,sizeof(double));
	memset(&integrated_speedyw,0,sizeof(double));

 	memset(&_odo_pose_acc, 0 , sizeof(pose_t));

	memset(&_odo_speed_acc, 0 , sizeof(pose_t));

	memset(&_odo_pose_enc, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
}
