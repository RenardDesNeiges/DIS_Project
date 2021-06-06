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
#define ROBOTS 1
#define BUFFER_SIZE 11
#define SWARMSIZE 11

/* Functions */

// Run particle swarm optimization
double* pso(
    void (*fitness_fn)(double particles[ROBOTS][BUFFER_SIZE],double fit[ROBOTS],int neighbors[SWARMSIZE][SWARMSIZE]),
    int n_swarmsize, int n_nb, double lweight, double nbweight, double vmax, double min,
    double max, int iterations, int n_datasize, int n_robots
);
