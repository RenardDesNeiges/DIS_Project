/*****************************************************************************/
/* File:         supervisor.c                             		             */
/* Date:         2021	   	 			                                     */
/* Description:  webots supervisor for consensus formation control that 	 */
/*				 implements PSO for searching through the hyperparameters  	 */
/*				 of the controller											 */
/* Author: 	 Titouan Renard												     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#include "../pso/pso.h"
#include "../communication/communication.h"
#include "../controller/controller.h"

#define ROBOT_NUMBER 5
#define SIM_TIME 500

/*CONSTANTES*/
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define TIME_STEP 64      // time step between measurement in milliseconds
#define WHEEL_RADIUS 	0.0201		// Radius of the wheel in meter

static WbNodeRef robs[ROBOT_NUMBER];
static WbFieldRef robs_translation[ROBOT_NUMBER];
static WbFieldRef robs_rotation[ROBOT_NUMBER];
WbDeviceTag emitter_device;

double start_angle[4] = {0.0,1.0,0.0,-1.570796};
double start_pose[4][3] = {{-2.9,0,-0.05},{-2.9,0,0.11},{-2.9,0,-0.2},{-2.9,0,0.26}};

double loc[ROBOT_NUMBER][4];
double avg_loc_team[4];
double old_avg_loc_team[4];
double Dfl = 0.15;

FILE *fp;

/* Good relative positions for each robot */
double good_rp[ROBOT_NUMBER][2] = { {0.0,0.0}, {0.0,-0.30}};

void reset_supervisor(void) {

	wb_robot_init();

	char rob[7] = "epuck0";
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
	emitter_device = wb_robot_get_device("emitter");

}

bool sup_init_log(const char* filename)
{

  fp = fopen(filename,"w");
  bool err = (fp == NULL);

  if( !err )
  {
	for(int i = 0; i< ROBOT_NUMBER; i++)
	{
		fprintf(fp, "x%d,y%d,",i,i);
	}
    fprintf(fp,"\n");
  }
  else
  {
	  printf("Fails to create a log file\n");
  }

  return err;
}

void sup_print_log()
{	
	double x,y;
	
	if( fp != NULL)
	{
		for(int i = 0; i<ROBOT_NUMBER;i++)
		{
			x = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
			y = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];
			fprintf(fp, "%f,%f,",x,y);
		}
	    fprintf(fp,"\n");
	}

}


void compute_orient_and_distance_score(double *fit_o, double *fit_d){
	int i,j;
	*fit_o = 1;
	double fit_d1 = 1;
	double fit_d2 = 0;
	double Hdiff = 0;
	double inter_robot_distance = 0;
	double N_pairs = ROBOT_NUMBER*(ROBOT_NUMBER-1)/2;


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

				/* compute orientation metric between robots*/

				if (i != j) {
					inter_robot_distance += sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][2]-loc[j][2],2));
					Hdiff = loc[i][3]-loc[j][3];
					*fit_o -= fabs(Hdiff)/(N_pairs*M_PI);
				}
			}

			fit_d1 += sqrt(pow(loc[i][0]-avg_loc_team[0],2) + pow(loc[i][2]-avg_loc_team[2],2));
		}

		fit_d1 = 1/fit_d1;

		for (i=0;i<ROBOT_NUMBER;i++) {    
			for (j=0;j<ROBOT_NUMBER;j++) {
				fit_d2 += fmin((inter_robot_distance/Dfl),(1.0/pow(1-Dfl+inter_robot_distance,2)));
			}
		}
		*fit_d = fit_d1*fit_d2;
}



double compute_vel_score(int cnt){
    /* Compute how fast we move */
    
    
    double max_distance = WHEEL_RADIUS*MAX_SPEED_WEB*TIME_STEP;
    
    /* move the average coords to the old one */
    if(cnt == 0){
        old_avg_loc_team[0] = 0;
        old_avg_loc_team[2] = 0;        
    }
    else{
        old_avg_loc_team[0] = avg_loc_team[0];
        old_avg_loc_team[2] = avg_loc_team[0];
    }
    avg_loc_team[0] = 0;
    avg_loc_team[2] = 0;  
    
    /* measure coordinates and copmute new avereage */
    for(int i=0;i<ROBOT_NUMBER;i++) {
        avg_loc_team[0] += wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
        avg_loc_team[2] += wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];        
    }
    avg_loc_team[0] /= ROBOT_NUMBER;
    avg_loc_team[2] /= ROBOT_NUMBER;
    double d_x = avg_loc_team[0] - old_avg_loc_team[0];
    double d_z = avg_loc_team[2] - old_avg_loc_team[2];
    double distance_moved = sqrt(d_x*d_x + d_z*d_z);  // we ignore rotation for this
      
    return distance_moved / max_distance; 
}


void set_hyperparameters(float* hyperparamters)
{
	float buffer[BUFFER_SIZE];

	for(int i = 0; i < BUFFER_SIZE; i++)
		buffer[i] = hyperparamters[i];

	wb_emitter_send(emitter_device,(float *)buffer,BUFFER_SIZE*sizeof(float));
}

void reset_positions()
{
	for(int i = 0; i < ROBOT_NUMBER; i++)
	{

		wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"), start_pose[i]);
		wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"), start_angle);
	}
}

void reset_simulation(float* hyperparamters)
{
	reset_positions();
	set_hyperparameters(hyperparamters);
}

double run_simulation(bool print_enabled, const char* filename){;
	int cnt;
	double o_score =0.0, v_score = 0.0, d_score = 0.0, score = 0.0, avg_score = 0.0, cost = 0.0;

	for(cnt = 0; cnt < SIM_TIME; cnt++) { /* The robot never dies! */
		/*compute the score at this time step*/
		sup_print_log();
		v_score = compute_vel_score(cnt);
		compute_orient_and_distance_score(&o_score, &d_score);
		printf("v: %f, o: %f, d: %f\n",v_score, o_score, d_score);
		score = o_score*v_score*d_score;  // we want to MAXIMIZE this
		printf("score: %f\n", score);
		avg_score += score;
		if (print_enabled)
                          printf("O_Score: %.2f, D_Score: %.2f, V_Score: %.2f, score: %.2f, Average score: %.2f\n", o_score, d_score, v_score, score, avg_score / cnt);

		wb_robot_step(64); /* run one step */
	}

	cost = avg_score/SIM_TIME;

	return cost;
}

int main(int argc, char *args[]) {
	
	int print_enabled = 1;
	double cost;
	reset_supervisor();
	char filename[11] = "trace_.csv";

	float hyperparameters[BUFFER_SIZE];
	hyperparameters[FLOCK_COHESION] = 0.15;
	hyperparameters[FLOCK_DISPERSION] = 0.002;
	hyperparameters[FLOCK_CONSISTENCY] = 0.01;
	hyperparameters[FLOCK_RULE2_RADIUS] = 0.15;
	hyperparameters[K_A] = 200;
	hyperparameters[K_B] = 500;
	hyperparameters[K_C] = 0.001;
	
	for(int i = 0; i<10; i++)
	{
		wb_robot_step(64); /* wait 10 steps for the robots to initialize */
	}

	for(int i = 0; i< 15; i++)
	{

		snprintf(filename, 11, "trace%d.csv", i);
		sup_init_log(filename);
		printf("Run %d\n", i);
		reset_simulation(hyperparameters);
		cost = run_simulation(print_enabled,filename);
		printf("metric = %f\n", cost);
		fclose(fp);
	}
	
	return 0;
}
