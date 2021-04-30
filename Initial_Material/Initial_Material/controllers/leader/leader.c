#include <math.h>
#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include "trajectories.h"
#include "odometry.h"
#include "kalman.h"
#include "localization.h"
//-----------------------------------------------------------------------------------//
/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  controller_error(X, Y, __LINE__, __FILE__)

/*CONSTANTES*/
#define MAX_SPEED 1000          // Maximum speed 
#define INC_SPEED 5             // Increment not expressed in webots 
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define TIME_INIT 0		        // Time in second

/*VERBOSE_FLAGS*/
#define VERBOSE_GPS false        // Print GPS values
#define VERBOSE_ACC false       // Print accelerometer values
#define VERBOSE_ACC_MEAN false  // Print accelerometer mean values
#define VERBOSE_POSE false      // Print pose values
#define VERBOSE_ENC true       // Print encoder values

#define NB_SENSORS 8
#define DETECT_RANGE 300
#define TIME_STEP 64

WbDeviceTag ps[NB_SENSORS]; // list of distance sensor handles


/*DEFINITIONS*/
typedef struct
{
  int time_step;
  WbDeviceTag left_motor; 
  WbDeviceTag right_motor; 
} simulation_t;




//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static simulation_t _robot;
//---------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------//
/*FUNCTIONS*/
static bool controller_init();
static bool controller_init_time_step();

static bool controller_init_motor();

static void controller_print_log(double time);


static bool controller_error(bool test, const char * message, int line, const char * fileName);



//-----------------------------------------------------------------------------------//

void compute_wheel_speeds(double u_x, double u_y, int *msl, int *msr) {
	
	// Compute the range and bearing to the wanted position
	double heading_error = atan2(u_y, u_x);
	double distance = sqrt(pow(u_x, 2) + pow(u_y, 2));
	// Convert to wheel speeds!
	*msl = (int) (distance-heading_error/M_PI)*MAX_SPEED_WEB;
	*msr = (int) (distance+heading_error/M_PI)*MAX_SPEED_WEB;
}



void init_prox_sensor(){
	int i;
	char name[] = "ps0";
	for(i = 0; i < NB_SENSORS; i++) {
		ps[i]=wb_robot_get_device(name); // get sensor handle
		// perform distance measurements every TIME_STEP millisecond
		wb_distance_sensor_enable(ps[i], TIME_STEP);
		name[2]++; // increase the device name to "ps1", "ps2", etc.
	}
}

void proximity_vector(double *ox, double *oy)
{
	int i;
	int sense = 0;
	static double ds_value[NB_SENSORS]; 
	//directions in which each sensors points
	float sx[8] = {0.2588,0.7071,1, 0.5736,-0.5736,-1,-0.7071,-0.2588};
	float sy[8] = {0.9659,0.7071,0,-0.8192,-0.8192, 0, 0.7071, 0.9659};

	*ox = 0;
	*oy = 0;	


	for (i = 0; i < NB_SENSORS; i++)
	{
		ds_value[i] = wb_distance_sensor_get_value(ps[i]);
		if(ds_value[i] > DETECT_RANGE)
			sense = 1;
		
		*ox = *ox + (ds_value[i]/1000)*sx[i];
		*oy = *oy + (ds_value[i]/1000)*sy[i];
	}

	*ox *= sense;
	*oy *= sense;

}

void get_navigation_vector(double *n_x, double *n_y, pose_t goal)
{
	double e_theta, e_dist;

	// Compute direction of the target point (norm saturates at 1)
	// The vector is first computed in global frame, in polar coordinates
	e_dist = sqrt(pow(goal.x - get_pose().x, 2)+pow(goal.y - get_pose().y, 2));
	e_theta = atan2(goal.y - get_pose().y, goal.x - get_pose().x);

	// Rotate the direction vector into the robot's frame
	e_theta -= get_pose().heading;

	// Saturate the norm of the vector at 1
	if(e_dist > 1)
		e_dist = 1;
	

	// Output the value in carthesian coordinates (in robot's frame)
	*n_x = e_dist * cos(e_theta);
	*n_y = e_dist * sin(e_theta);
}

int main() 
{

	// Goal point (this is where the leader goes)
	pose_t goal;
	goal.x = 0;
	goal.y = -10;
	goal.heading = 0;

	// Coordinates representing direction of goal point (navigation vector)
	double n_x;
	double n_y;

	// Coordinates representing direction of sensor obstacles (Braitenberg-esque)
	double s_x;
	double s_y;

	// Coordinates representing direction of sensor movement (control law)
	double u_x;
	double u_y;
	double u_norm; //norm of the control law (in practice saturates at 1)

	// Left and right motor speeds
	int msl_w;
	int msr_w;

	

	wb_robot_init();	
	init_prox_sensor();

	if(CATCH_ERR(controller_init(), "Controller fails to init \n"))
		return 1;

	while (wb_robot_step(_robot.time_step) != -1) 
	{
		if(wb_robot_get_time() < TIME_INIT)
		{
			loc_calibrate(TIME_INIT,_robot.time_step);
		}
		else
		{
			// 1. Perception / Measurement
			loc_update_measures();
			// 2. Compute pose
			loc_compute_pose();
			
			get_navigation_vector(&n_x, &n_y, goal);
			// control law is a linear combination of goal and sensors


			proximity_vector(&s_x, &s_y); // get sensor proximity vectorproximity_vector(&u_x, &u_y); // get sensor proximity vector

			// Control vector is a linear combination of navigation and sensors for obstacle avoidance
			u_x = n_x-s_x;
			u_y = n_y-s_y;

			// Saturate the norm of command vector at 1
			u_norm = sqrt(pow(u_x, 2) + pow(u_y, 2));
			if(u_norm>1){
				u_x /= u_norm;
				u_y /= u_norm;
			}

			printf("%f , %f\n", u_x, u_y);


			// outputing the control vector to the wheel speed (vector must of of norm <=1)
			compute_wheel_speeds(u_x, u_y,  &msl_w, &msr_w);
			wb_motor_set_velocity(_robot.left_motor, msl_w);
			wb_motor_set_velocity(_robot.right_motor, msr_w);
			
			// Logging Step
			controller_print_log(wb_robot_get_time());

		}
	}



}







//INIT FUNCTIONS
/**
 * @brief      Run the initialization. Set the variables and structure to 0. Try to initialize the Webots components. 
 * 					 note that only the required components are initialized to prevent involontary useage
 *
 * @return     Return true if it rise an error
 */
bool controller_init()
{

	bool err = false;
	pose_t pose_origine = {-2.9,0.,0.};
	memset(&_robot, 0 , sizeof(simulation_t));
	CATCH(err,controller_init_time_step());
	CATCH(err,loc_init(_robot.time_step, pose_origine));
	CATCH(err,controller_init_motor());
	return err;
}

/**
 * @brief      Initialize the simulation time step on Webots
 *
 * @return     return true if it fails
 */
bool controller_init_time_step()
{

  _robot.time_step = wb_robot_get_basic_time_step();

  return CATCH_ERR(_robot.time_step == 0,"time step is not is not set\n");
}


/**
 * @brief      Initiliaze the the motor from Webots
 *
 * @return      return true if it fails
 */
static bool controller_init_motor() {

	_robot.left_motor = wb_robot_get_device("left wheel motor");
	bool err = CATCH_ERR(_robot.left_motor == 0, "No left wheel motor node found in the current robot file\n");

	_robot.right_motor = wb_robot_get_device("right wheel motor");
	CATCH(err,CATCH_ERR(_robot.right_motor == 0, "No right wheel motor node found in the current robot file\n"));
	if (!err)
	{
		wb_motor_set_position(_robot.left_motor, INFINITY);
		wb_motor_set_position(_robot.right_motor, INFINITY);
		wb_motor_set_velocity(_robot.left_motor, 0.0);
		wb_motor_set_velocity(_robot.right_motor, 0.0);
	}
	return err;
}


//LOG FUNCTIONS
/**
 * @brief	centralize the log printing step
 */
static void controller_print_log(double time)
{
	loc_print_log(time);
}


//ERROR FUNCTIONS
/**
 * @brief      Do an error test if the result is true write the message in the stderr.
 *
 * @param[in]  test     The error test to run
 * @param[in]  message  The error message
 *
 * @return     true if there is an error
 */
bool controller_error(bool test, const char * message, int line, const char * fileName)
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
