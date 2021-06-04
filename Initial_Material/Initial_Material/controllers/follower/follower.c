#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include "../localization_controller/odometry.h"
#include "../localization_controller/localization.h"
#include "../controller/controller.h"
//-----------------------------------------------------------------------------------//
/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  controller_error(X, Y, __LINE__, __FILE__)

/*CONSTANTES*/
#define MAX_SPEED 1000          // Maximum speed 
#define INC_SPEED 5             // Increment not expressed in webots 
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
// #define TIME_INIT 5		        // Time in second
#define TIME_INIT 1		        // Time in second
#define WHEEL_RADIUS 20.5
double wheel_speed_threshold = 6.275*0.5; //max value minus a small epsilon, lowered so that the followers can follow


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
pose_t migration_vector;	// control vector of the migratory urge
pose_t obstacle_avoidance_vector;	// control vector of the migratory urge
pose_t consensus_vector;			// control vector of the migratory urge
pose_t migration_goal;		// goal of migratory urge
pose_t control_vector;		// summed_up control vector
double u_omega, u_v;		// unicycle model control vector
double w_left, w_right; 	// left and right wheel speeds
double ka = 100;
double kb = 30;
double kc = 0.001;
double kp =30;
double ki = 0.5;
double w[ROBOT_NUMBER] = {5, .5, .5, .5};
int robot_id;
pose_t goal_pose[ROBOT_NUMBER];		// control vector of the migratory urge
//---------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------//
/*FUNCTIONS*/
static bool controller_init();
static bool controller_init_time_step();
static bool controller_init_motor();
static void controller_print_log(double time);
static bool controller_error(bool test, const char * message, int line, const char * fileName);

void leader_update();



//-----------------------------------------------------------------------------------//



int main() 
{	
	migration_goal.x = 4;
	migration_goal.y = 0;
	migration_goal.heading = 0;
	wb_robot_init();	
	init_prox_sensor();

	

	if(CATCH_ERR(controller_init(), "Controller fails to init \n"))
		return 1;

	while (wb_robot_step(_robot.time_step) != -1) 
	{
		if(wb_robot_get_time() < TIME_INIT)
		{
			loc_calibrate(TIME_INIT,_robot.time_step);
			controller_print_log(wb_robot_get_time());
		}
		else
		{
			// 1. Perception / Measurement
			loc_update_measures();
			// 2. Compute pose
			loc_compute_pose();
			
			// 3. Control
			leader_update();
		}
	}



}

void leader_update()  
{
	//main update function

	migration_urge(&migration_vector, loc_get_pose(), migration_goal); 			// get the migration vector (stored in the migration_vector global variable)
	local_avoidance_controller(&obstacle_avoidance_vector, loc_get_pose()); 			// get the obstacle avoidance vector (stored in the obstacle_avoidance_vector global variable)
	// printf("ox = %f, oy = %f \Nur ", obstacle_avoidance_vector.x, obstacle_avoidance_vector.y);

	consensus_controller(&consensus_vector, loc_get_pose(), goal_pose, kp, ki, robot_id, w);

	control_vector = pose_add_3( pose_scale(0.005, obstacle_avoidance_vector), pose_scale(0,migration_vector), pose_scale(0.5,consensus_vector));

	unicycle_controller(&u_omega, &u_v, loc_get_pose(), control_vector, ka, kb, kc); 	// get the unicycle control law from the computed global movement vector,  (stored in the unicycle_control global variable)
	unicylce_to_wheels(&w_left, &w_right, u_omega, u_v,WHEEL_RADIUS,wheel_speed_threshold); 					// get the wheel speeds from the unicylce control law
	wb_motor_set_velocity(_robot.left_motor, w_left);
	wb_motor_set_velocity(_robot.right_motor, w_right);

	// printf("x = %f, y = %f, heading =  %f \n", loc_get_pose().x, loc_get_pose().y, loc_get_pose().heading);

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
	init_range_bearing_estimates(&robot_id, goal_pose);
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

    fprintf(stderr,"%s",buffer);

    return(true);
  }

  return false;
}
