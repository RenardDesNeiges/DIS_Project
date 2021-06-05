/**************************************************/
/*          Particle Swarm Optimization           */
/*          Header file                           */
/*                                                */
/*          Author: Jim Pugh                      */
/*          Last Modified: 1.10.04                */
/*                                                */
/**************************************************/

#define FONT "Arial"

#include "../communication/communication.h"

#define NB_SENSOR 8
#define SWARMSIZE 14

/* Functions */

// Run particle swarm optimization
double* pso(int n_swarmsize, int n_nb, double lweight, double nbweight, 
            double vmax, double min, double max, int iterations, int n_datasize, 
            int n_robots); 

// Fitness function for particle evolution
void fitness(double particles[SWARMSIZE][BUFFER_SIZE],double fit[SWARMSIZE],int neighbors[SWARMSIZE][SWARMSIZE]);                       

// Find the current performance of the swarm
void findPerformance(double swarm[SWARMSIZE][BUFFER_SIZE], double perf[SWARMSIZE],
		     double age[SWARMSIZE], char type, int robots,
		     int neighbors[SWARMSIZE][SWARMSIZE]);  

// Update the best performance of a single particle
void updateLocalPerf(double swarm[SWARMSIZE][BUFFER_SIZE], double perf[SWARMSIZE], 
                double lbest[SWARMSIZE][BUFFER_SIZE], double lbestperf[SWARMSIZE], 
                double lbestage[SWARMSIZE]);

// Copy value of one particle to another
void copyParticle(double particle1[BUFFER_SIZE], 
                double particle2[BUFFER_SIZE]);                            

// Update the best performance of a particle neighborhood
void updateNBPerf(double lbest[SWARMSIZE][BUFFER_SIZE], double lbestperf[SWARMSIZE],
		      double nbbest[SWARMSIZE][BUFFER_SIZE], double nbbestperf[SWARMSIZE],
		      int neighbors[SWARMSIZE][SWARMSIZE]);

// Find the best result in a swarm
double bestResult(double lbest[SWARMSIZE][BUFFER_SIZE], double lbestperf[SWARMSIZE], 
                double best[BUFFER_SIZE]);

/* math utility functions */

// Generate random number in [0,1]
double rnd(void);     

// Modulus function
int mod(int,int);

// S-function to transform [-infinity,infinity] to [0,1]
double s(double);        


