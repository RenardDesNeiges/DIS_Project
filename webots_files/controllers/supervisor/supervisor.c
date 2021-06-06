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

// #include "../pso/pso.h"
#include "../communication/communication.h"
#include "../controller/controller.h"

#define ROBOT_NUMBER 4
#define SIM_TIME 500

/*CONSTANTES*/
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define TIME_STEP 64      // time step between measurement in milliseconds
#define WHEEL_RADIUS 	0.0201		// Radius of the wheel in meter


/* PSO definitions */
#define SWARMSIZE 10                    // Number of particles in swarm (defined in pso.h)
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 40.0                       // Maximum velocity particle can attain
#define MIN_EXP_INIT -5.0                   // Lower bound on initialization value
#define MAX_EXP_INIT 5.0                    // Upper bound on initialization value
#define ITS 20                          // Number of iterations to run
#define MAX_ROB 4                      // Maximum number of parallel devices

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 180                     // Number of fitness steps to run during optimization

#define FINALRUNS 1
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8



static WbNodeRef robs[ROBOT_NUMBER];
static WbFieldRef robs_translation[ROBOT_NUMBER];
static WbFieldRef robs_rotation[ROBOT_NUMBER];
WbDeviceTag emitter_device;

double start_angle[4] = {0.0,1.0,0.0,-1.570796};
double start_pose[4][3] = {{-2.9,0,-0.05},{-2.9,0,0.11},{-2.9,0,-0.2},{-2.9,0,0.26}};

double loc[ROBOT_NUMBER][4];
double avg_loc_team[4];
double old_avg_loc_team[4];

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


double compute_pos_score(int cnt, double* rel_goal_x,double* rel_goal_z){
	int i,j;
	double err = 0.0;
	double goal_x, goal_z;

	float rel_x,rel_z;
      	for (i=0;i<1;i++) {    /* Compute error only with respect to leader */
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
			/* If this is the first measure, save it for computation of the error later */
				if(cnt == 0)
				{
					rel_goal_x[j] = rel_x;
					rel_goal_z[j] = rel_z;
				}

				double e_x, e_z;

				/* Check error in position of robot */
				e_x = rel_x - rel_goal_x[j];
				e_z = rel_z - rel_goal_z[j];
				err += sqrt(e_x*e_x + e_z*e_z);

		}
	}
            return 1 / (1 + (err / ROBOT_NUMBER));
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




double compute_terminal_error(){
	int i;
	double rx, rz, e_x, e_z;
	double err = 0.0;
	for (i=0;i<ROBOT_NUMBER;i++) {
		rx = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
		rz = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];
		e_x = rx-MIGRATION_X;
		e_z = rz-MIGRATION_Y;
		err += e_x*e_x + e_z*e_z; //apparently multiplicaitons are much faster than pow
	}
	return err;
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


double run_simulation(int logging_level){
	int cnt;
	double d_score =0.0, v_score = 0.0, score = 0.0, avg_score = 0.0, cost = 0.0;
	double rel_goal_x[ROBOT_NUMBER], rel_goal_z[ROBOT_NUMBER];

	for(cnt = 0; cnt < SIM_TIME; cnt++) { /* The robot never dies! */
		/*compute the score at this time step*/
		if(logging_level > 0)
                            sup_print_log();
		d_score = compute_pos_score(cnt, rel_goal_x, rel_goal_z);
		v_score = compute_vel_score(cnt);
		score = d_score*v_score;  // we want to MAXIMIZE this
		avg_score += score;
		if (logging_level > 1)
                          printf("D_Score: %.2f, V_Score: %.2f, score: %.2f, Average score: %.2f\n", d_score, v_score, score, avg_score / cnt);

		

		wb_robot_step(64); /* run one step */
	}
	printf("(%f,%f), ", avg_score/SIM_TIME, compute_terminal_error()/30);
	cost = avg_score/SIM_TIME;

	return cost;
}

double calc_fitness(float* hyperparameters, int num_its){
    reset_simulation(hyperparameters);
    double score = run_simulation(0);
        // printf("value of score was %.2f /n", score);
    return score;

}


void run_pso(float* hyperparameters, int n_swarmsize, int n_nb,
          double lweight, double nbweight, double vmax, double min,
          double max, int iterations, int n_datasize){
    float def_hyperparameters[BUFFER_SIZE];
	def_hyperparameters[ALPHA] = 100.0;
	def_hyperparameters[BETA_L] = 0.0;
	def_hyperparameters[BETA_F] = 1.0;
	def_hyperparameters[THETA_L] = 1.0;
	def_hyperparameters[THETA_F] = 0.0;
	def_hyperparameters[LAMBDA] = 10.0;
	def_hyperparameters[IOTA] = 0.0005;
	def_hyperparameters[K_A] = 200;
	def_hyperparameters[K_B] = 500;
	def_hyperparameters[K_C] = 0.001;
	def_hyperparameters[EPSILON_L] = 0.4;
	
  for (int i = 0; i < BUFFER_SIZE; i++){
	       hyperparameters[i] = def_hyperparameters[i];
      }
}

int main(int argc, char *args[]) {
	


	
    printf("Particle Swarm Optimization Super Controller\n");
	reset_supervisor();
    	
    wb_robot_step(10*TIME_STEP); /* wait for the robots to initialize */
	
	
    printf("Initialized\n");
    

        float hyperparameters[BUFFER_SIZE];                         // Current params
    float best_hyperparameters[BUFFER_SIZE];                    // Best params
    int i,j,k;                               // Counter variables
    

  double fit;                        // Fitness of the current FINALRUN
  double endfit;                     // Best fitness over 10 runs
  double bestfit;
  /* Evolve controllers */
  endfit = 0.0;
  bestfit = 0.0;

  // Do 10 runs and send the best controller found to the robot
  for (j=0;j<10;j++) {
    // Get result of optimization
    run_pso(hyperparameters, SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MAX_EXP_INIT,MIN_EXP_INIT,ITS,BUFFER_SIZE);

    // Run FINALRUN tests and calculate average
    
    fit = 0.0;
    for (i=0;i<FINALRUNS;i++) {
        fit += calc_fitness(hyperparameters,FIT_ITS);
    }
    fit /= FINALRUNS;

    // Check for new best fitness
    if (fit > bestfit) {
      bestfit = fit;
      for (i = 0; i < BUFFER_SIZE; i++){
	       best_hyperparameters[i] = hyperparameters[i];
      }
    }

    printf("\n Performance of the best solution in iteration %d: %.3f\n", j, fit);
    endfit += fit/10; // average over the 10 runs
  }
    
	
    printf("~~~~~~~~ Optimization finished.\n");
  printf("Best performance: %.3f\n",bestfit);
  printf("Average performance: %.3f\n",endfit);
  
  /* Run with best hyperaparameters*/
  int logging_level = 1;
  char filename[11] = "trace_best.csv";
  // snprintf(filename, 11, "trace_best.csv");
  sup_init_log(filename);
  reset_simulation(best_hyperparameters);
  run_simulation(logging_level);
  fclose(fp);
    
	return 0;
}

