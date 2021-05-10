

#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  7	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)


#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (1.0/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction



void reynolds_rules() {
	int i, j, k;			// Loop counters
	float avg_loc[2] = {0,0};	// Flock average positions
	float avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	float dist;
	float n_robots;
	
	
	/* Compute averages over the whole flock */
        n_robots =1;
        for(i=0; i<FLOCK_SIZE; i++) {
          if (i == robot_id) {
            continue;
          }
          dist = sqrt(pow(loc[i][0]-loc[robot_id][0],2.0)+pow(loc[i][1]-loc[robot_id][1],2.0));
          if (dist < 0.4){
                  for (j=0;j<2;j++){
                      avg_speed[j] += speed[i][j];
                      avg_loc[j] += loc[i][j];
          
                      }
                      n_robots = n_robots+1;
                  }
           }
             
             for (j=0;j<2;j++){
             if (n_robots>1){
                 avg_speed[j] /= (n_robots-1);
                 avg_loc[j] /= (n_robots-1);
              }
              else{
              avg_speed[j] = speed[robot_id][j];
              avg_loc[j] = loc[robot_id][j];
              }
           }
         
	/* Reynold's rules */
	
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	for (j=0;j<2;j++) {
		if (sqrt(pow(loc[robot_id][0]-avg_loc[0],2)+pow(loc[robot_id][1]-avg_loc[1],2)) > RULE1_THRESHOLD) {
         		cohesion[j] = avg_loc[j] - loc[robot_id][j];
		}
	}
	
	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id) {
			if (pow(loc[robot_id][0]-loc[k][0],2)+pow(loc[robot_id][1]-loc[k][1],2) < RULE2_THRESHOLD) {
				for (j=0;j<2;j++) {
					dispersion[j] += 1/(loc[robot_id][j] -loc[k][j]);
				}
			}
		}
	}
  
	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	consistency[0] = 0;
	consistency[1] = 0;
	for (j=0;j<2;j++) {
		consistency[j] = avg_speed[j] - speed[robot_id][j]; 
	}

	// aggregation of all behaviors with relative influence determined by weights
	
	for (j=0;j<2;j++) {
		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
		speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
	}
	speed[robot_id][1] *= -1; //y axis of webots is inverted
	
	speed[robot_id][0] += MIGRATION_WEIGHT*(migr[0]- loc[robot_id][0]);
    speed[robot_id][1] -= MIGRATION_WEIGHT*(migr[1]- loc[robot_id][1]); //y axis of webots is inverted

	}
}


void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}

