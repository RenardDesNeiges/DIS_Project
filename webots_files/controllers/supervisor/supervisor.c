/*****************************************************************************/
/* File:         supervisor.c                             		             */
/* Version:      2.0                                                         */
/* Date:         10-Oct-14 -- 06-Oct-2015                                    */
/* Description:  The supervisor of a flock of robots which takes care of     */
/*				 sending the positions of the robots to the mates     	     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi 									     */
/*				 initially developed by Nicolas Correll  		  		     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#include "../pso/pso.h"
#include "../communication/communication.h"

#define ROBOT_NUMBER 4
#define SIM_TIME 400

static WbNodeRef robs[ROBOT_NUMBER];
static WbFieldRef robs_translation[ROBOT_NUMBER];
static WbFieldRef robs_rotation[ROBOT_NUMBER];
WbDeviceTag emitter_device;

double loc[ROBOT_NUMBER][4];

/* Good relative positions for each robot */
double good_rp[ROBOT_NUMBER][2] = { {0.0,0.0}, {0.0,-0.30}};

void reset_supervisor(void) {

	wb_robot_init();

	char rob[7] = "epuck0";
	char emitter0[8] = "emitter";
	int i;
	robs[0] = wb_supervisor_node_get_from_def(rob);


	robs_translation[0] = wb_supervisor_node_get_field(robs[0],"translation");
	robs_rotation[0] = wb_supervisor_node_get_field(robs[0],"rotation");

	rob[5]++;
	for (i=1;i<ROBOT_NUMBER;i++) {

		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_translation[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");    
		rob[5]++;
	}
	emitter_device = wb_robot_get_device(emitter0);

}

double compute_pos_error(int cnt, double* rel_goal_x,double* rel_goal_z){
	int i,j;
	double err = 0.0;
	double goal_x, goal_z;

	float rel_x,rel_z;
	for (i=0;i<ROBOT_NUMBER;i++) {
		for (j=0;j<ROBOT_NUMBER;j++) {

			/* Get data */
			loc[j][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[j])[0];
			loc[j][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[j])[1];
			loc[j][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[j])[2];
			loc[j][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[j])[3];
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[1];
			loc[i][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];
			loc[i][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3];

			/* Find global relative coordinates */

			goal_x = loc[i][0] - loc[j][0];
			goal_z = loc[i][2] - loc[j][2];
			/* Calculate relative coordinates */
			rel_x = -goal_x*cos(loc[i][3]) + goal_z*sin(loc[i][3]);
			rel_z =  goal_x*sin(loc[i][3]) + goal_z*cos(loc[i][3]);
			
			/* error metric computation */
			if(i == 0)
			{
				/* If this is the first measure, save it for computation of the error later */
				if(cnt == 0)
				{
					rel_goal_x[j] = rel_x;
					rel_goal_z[j] = rel_z;
				}

				double e_x, e_z;

				/* Check error in position of robot */
				e_x = pow(rel_x - rel_goal_x[j],2);
				e_z = pow(rel_z - rel_goal_z[j],2);
				err += e_x + e_z;
			}

		}
	}
	return err;
}

double run_simulation(print_enabled){;
	int cnt;
	double err = 0.0, avg_err = 0.0;
	double rel_goal_x[ROBOT_NUMBER], rel_goal_z[ROBOT_NUMBER];
	for(cnt = 0; cnt < SIM_TIME; cnt++) { /* The robot never dies! */
		/*compute the error at this time step*/
		err = compute_pos_error(cnt, rel_goal_x, rel_goal_z);
		avg_err += err;
		if (print_enabled)
			printf("Error: %.2f, Average Error: %.2f\n",err,avg_err/cnt);

		wb_robot_step(64); /* run one step */
	}
	return avg_err;
}

int main(int argc, char *args[]) {
	
	int print_enabled = 1;

	reset_supervisor();

	for(int i = 1; i< 5; i++)
	{
		run_simulation(print_enabled);
	}
	
	return 0;
}

