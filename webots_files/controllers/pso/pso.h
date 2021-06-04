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
                                        // + 2 recursive/lateral conenctions + 1 bias)
#define SWARMSIZE 12

// Functions
double* pso(int,int,double,double,double,double,double,int,int,int); // Run particle swarm optimization
void fitness(double[][BUFFER_SIZE],double[],int[][SWARMSIZE]);                       // Fitness function for particle evolution
double rnd(void);                                                // Generate random number in [0,1]
void findPerformance(double[][BUFFER_SIZE],double[],double[],char,int,int[][SWARMSIZE]);  // Find the current performance of the swarm
void updateLocalPerf(double[][BUFFER_SIZE],double[],double[][BUFFER_SIZE],double[],double[]);   // Update the best performance of a single particle
void copyParticle(double[],double[]);                            // Copy value of one particle to another
void updateNBPerf(double[][BUFFER_SIZE],double[],double[][BUFFER_SIZE],double[],int[][SWARMSIZE]);  // Update the best performance of a particle neighborhood
int mod(int,int);                                                // Modulus function
double s(double);                                                // S-function to transform [-infinity,infinity] to [0,1]
double bestResult(double[][BUFFER_SIZE],double[],double[]);                 // Find the best result in a swarm
