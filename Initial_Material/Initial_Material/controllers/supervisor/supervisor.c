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

static WbNodeRef robs[ROBOT_NUMBER];
static WbFieldRef robs_translation[ROBOT_NUMBER];
static WbFieldRef robs_rotation[ROBOT_NUMBER];
WbDeviceTag emitter_device;

double loc[ROBOT_NUMBER][4];

/* Good relative positions for each robot */
double good_rp[ROBOT_NUMBER][2] = { {0.0,0.0}, {0.0,-0.30}};

void reset(void) {

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

int main(int argc, char *args[]) {
	float buffer[255];
	float global_x,global_z,rel_x,rel_z, rel_range, rel_bearing;
	double goal_x, goal_z;
	double temp_err, err, avg_err;
	int cnt,i,j;
	int print_enabled = 1;
	int send_interval = 5;

	double rel_goal_x[ROBOT_NUMBER], rel_goal_z[ROBOT_NUMBER];
	
	if (argc > 1) {
		print_enabled = atoi(args[1]);
		printf("Print: %d\n", print_enabled);
	}
	if (argc > 2) {
		send_interval = atoi(args[2]);
		if (send_interval < 1) send_interval = 1;
		if (send_interval > 1000) send_interval = 1000;
		printf("Sending at intervals of %d\n", send_interval);
	}

	reset();
	avg_err = 0.0;
	for(cnt = 0; ; cnt++) { /* The robot never dies! */

		/* Send relative positions to followers */
		err = 0.0;
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
				if(i == j)
				{
					rel_range = 0;
					rel_bearing = 0;
				}
				else
				{
					goal_x = loc[i][0] - loc[j][0];
					goal_z = loc[i][2] - loc[j][2];
					/* Calculate relative coordinates */
					rel_x = -goal_x*cos(loc[i][3]) + goal_z*sin(loc[i][3]);
					rel_z = goal_x*sin(loc[i][3]) + goal_z*cos(loc[i][3]);
					rel_range = sqrt(rel_x * rel_x + rel_z * rel_z);
					rel_bearing = -atan2(rel_x, rel_z);
				}
				
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



				/* Pack the data in a buffer and send it to the robots */
				buffer[PACKET_TYPE]= SENSOR;
				buffer[ID_I]= i;
				buffer[ID_J]= j;
				buffer[RANGE] = rel_range;
				buffer[BEARING] = rel_bearing;
				if (cnt % send_interval == 0){
					wb_emitter_send(emitter_device,(char *)buffer,BUFFER_SIZE*sizeof(float));        
					
				}
			}

			
		}

		avg_err += err;
		if (print_enabled)
			printf("Error: %.2f, Average Error: %.2f\n",err,avg_err/cnt);

		wb_robot_step(64); /* run one step */
	}
	return 0;
}
