/*
 * File:          localization_supervisor.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

 /*MACRO*/


#define TIME_STEP 64
// 1 to display the position
#define VERBOSE 0
// 1 to save the logs in a file
#define LOGS 1
//name of the saved logs
#define FILENAME "datatrue"
WbNodeRef rob;		// Robot nodes
WbFieldRef rob_trans;  	// Robots translation fields
WbFieldRef rob_rotation;	// Robots rotation fields
float loc[3];		// Location of everybody in the flock
static FILE *fp;

void reset(void);
bool supervisor_init_log(const char* filename);
void supervisor_print_log(double time);

/*
 * Initialize flock position and devices
 */
void reset(void) {

  wb_robot_init();
  
  rob = wb_supervisor_node_get_from_def("ROBOT1");
  rob_trans = wb_supervisor_node_get_field(rob,"translation");
  rob_rotation = wb_supervisor_node_get_field(rob,"rotation");
 
}
bool supervisor_init_log(const char* filename)
{
  bool fileopen = false;
  fp = fopen(filename,"w");
  


  if( !fp )
  {
     fileopen = true;
     fprintf(fp, "time; pose_x; pose_y; pose_heading;\n");
  }

  return fileopen;
}
void supervisor_print_log(double time)
{

  if( fp != NULL)
  {
    fprintf(fp, "%g; %g; %g; %g;\n",
            time, loc[0], loc[1], loc[2]);
  }

}

int main(int argc, char **argv) {
  
  /* necessary to initialize webots stuff */
  reset();
  if (LOGS ==1 ) {
    supervisor_init_log(FILENAME);
  }
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    loc[0] = wb_supervisor_field_get_sf_vec3f(rob_trans)[0]; // X
    loc[1] = wb_supervisor_field_get_sf_vec3f(rob_trans)[2]; // Z
    loc[2] = wb_supervisor_field_get_sf_rotation(rob_rotation)[3]; // THETA
    
    if(VERBOSE ==1){
      printf("pose of robot 0:   x:   %f   z:   %f   th:   %f \n ", loc[0], loc[1],loc[2]);
    }
    if(LOGS == 1) {
      supervisor_print_log(wb_robot_get_time());
    }
  };

  /* Enter your cleanup code here */
  if(fp != NULL)
    fclose(fp);
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
