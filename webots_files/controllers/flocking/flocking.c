#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h> 
#include <webots/position_sensor.h>
#include "../localization_controller/odometry.h"
#include "../localization_controller/localization.h"
#include "../communication/communication.h"
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
double wheel_speed_threshold = WS_THRESH; //max value minus a small epsilon, lowered so that the followers can follow


/*DEFINITIONS*/
typedef struct
{
  int time_step;
  WbDeviceTag left_motor; 
  WbDeviceTag right_motor; 
} simulation_t;




//-----------------------------------------------------------------------------------//

/* webots variables */
static simulation_t _robot;				// webots robot handler
int robot_id;							// id of the robot in webots

/* controller variables */

pose_t migration_vector;				// control vector of the migratory urge
pose_t reynolds_vector;				// control vector of the consensus controller urge
pose_t migration_goal;					// goal of migratory urge
pose_t control_vector;					// summed_up control vector
pose_t goal_pose[ROBOT_NUMBER];			// control poses of the consensus controller
double u_omega, u_v;					// unicycle model control vector
double w_left, w_right; 				// left and right wheel speeds

/* hyperparameters and pso */

double hyperparameters[BUFFER_SIZE];	// hyperparameter vector (buffer from supervisor for PSO)

/* hyperparameters, in PSO those are set according to the hyperparameter buffer */
double alpha = 0.005;					// obstacle avoidance weight
double beta = 1.0;						// reynolds
double theta = 1.0;						// migration vector weight
double lambda = 10.0;					// leader bias in consensus vector
double iota = 0.005;						// integrator term weight in consensus
double ka = 100;						// ka term of unicyle controller (see report for details)
double kb = 50;							// kb term of unicyle controller (see report for details)
double kc = 0.001;						// kc term of unicyle controller (see report for details)
double kp = 1.0;						// proportional term of consensus (redundant with beta --> 1)
double w[ROBOT_NUMBER]; 				// weight matrix collumn of consensus controller

double w_cohesion = 0.015;
double w_dispersion = 0.002;
double w_consistency = 0.1;
double rule2radius = 0.15;


//---------------------------------------------------------------------------------------//

/* functions */
static bool controller_init();
static bool controller_init_time_step();
static bool controller_init_motor();
static void controller_print_log(double time);
static bool controller_error(bool test, const char * message, int line, const char * fileName);
void set_variables_to_hyperparameters();
void control_update();



//-----------------------------------------------------------------------------------//


int main() 
{	
	
	
	int reset;

	
	for(;;)
	{	
		printf("reset\n");
		migration_goal.x = MIGRATION_X;
		migration_goal.y = MIGRATION_Y;
		migration_goal.heading = 0;
		wb_robot_init();	
		init_prox_sensor();
		if(CATCH_ERR(controller_init(), "Controller fails to init \n"))
			return 1;

		reset = 0;

		while (wb_robot_step(_robot.time_step) != -1 && reset == 0) 
		{
			if(wb_robot_get_time() < TIME_INIT)
			{
				loc_calibrate(TIME_INIT,_robot.time_step);
				controller_print_log(wb_robot_get_time());
			}
			else
			{
				#ifdef PSO	
				reset = get_hyperparameters_from_supervisor(hyperparameters); // getting a hyperparamater update resets the controller
				set_variables_to_hyperparameters();
				#endif // 

				// 1. Perception / Measurement
				loc_update_measures();
				// 2. Compute pose
				loc_compute_pose();
				
				// 3. Control
				control_update();


				controller_print_log(wb_robot_get_time());

			}
		}
	}
	



}

void set_variables_to_hyperparameters()
{
	alpha = hyperparameters[ALPHA];
	beta = hyperparameters[BETA_L];
	theta = hyperparameters[THETA_L];
	lambda = hyperparameters[LAMBDA];  w[0] = lambda; //leader bias
	iota = hyperparameters[IOTA];
	ka = hyperparameters[K_A];
	kb = hyperparameters[K_B];
	kc = hyperparameters[K_C];
	wheel_speed_threshold = WS_THRESH * hyperparameters[EPSILON_L];
}

void control_update()  
{
	//main update function

	migration_urge(&migration_vector, loc_get_pose(), migration_goal); 			// get the migration vector (stored in the migration_vector global variable)

	// consensus_controller(&consensus_vector, loc_get_pose(), goal_pose, kp, iota, robot_id, w);
	reynolds_controller(&reynolds_vector, loc_get_pose(), w_cohesion,w_dispersion,w_consistency, robot_id, rule2radius);

	control_vector = pose_add(pose_scale(theta,migration_vector), pose_scale(beta,reynolds_vector));
	

	unicycle_controller(&u_omega, &u_v, loc_get_pose(), control_vector, ka, kb, kc); 	// get the unicycle control law from the computed global movement vector,  (stored in the unicycle_control global variable)
	unicylce_to_wheels(&w_left, &w_right, u_omega, u_v,WHEEL_RADIUS,wheel_speed_threshold); 					// get the wheel speeds from the unicylce control law
	wb_motor_set_velocity(_robot.left_motor, w_left);
	wb_motor_set_velocity(_robot.right_motor, w_right);

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
	for(int i = 0; i<ROBOT_NUMBER;i++)
		w[i] = 1;
	pose_t pose_origine = {-2.9,0.,0.};
	memset(&_robot, 0 , sizeof(simulation_t));
	CATCH(err,controller_init_time_step());
	CATCH(err,loc_init(_robot.time_step, pose_origine));
	CATCH(err,controller_init_motor());
	init_range_bearing_estimates(&robot_id, goal_pose);
	#ifdef PSO	
	init_pso_reciever();
	#endif // 
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
