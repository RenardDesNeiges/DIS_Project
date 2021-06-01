#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include "trajectories.h"
#include "odometry.h"
#include "localization.h"
//-----------------------------------------------------------------------------------//
/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  controller_error(X, Y, __LINE__, __FILE__)

/*CONSTANTES*/
#define MAX_SPEED 1000          // Maximum speed 
#define INC_SPEED 5             // Increment not expressed in webots 
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define TIME_INIT 5		        // Time in second



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



int main() 
{
	wb_robot_init();	
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
			
			// Use one of the two trajectories.
			//trajectory_1(_robot.left_motor, _robot.right_motor);
			//    trajectory_2(dev_left_motor, dev_right_motor);
			trajectory_1_delay(_robot.left_motor, _robot.right_motor,TIME_INIT);
			//circle_delay(_robot.left_motor, _robot.right_motor,TIME_INIT) ;
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

    fprintf(stderr,"%s",buffer);

    return(true);
  }

  return false;
}
