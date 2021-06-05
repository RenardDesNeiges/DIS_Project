#ifndef KALMAN_H
#define KALMAN_H 




//specific constants used in this project

//use this mode if the accelerometer is used as a control input to 
//estimate the future pose. 
#define ACC_CONTROLLED 0

//use this mode if the accelerometer is used as a control input to 
//estimate the future pose, but the filter is given new state matrices at each steps
#define ACC_CONTROLLED_UPG 2

//use this to consider the accelerometer as a sensor, and to feed 
//directly the command to the actuators. This mode also use the kalman filter
//on the heading
#define VEL_CONTROLLED 1



#define MODE_KAL ACC_CONTROLLED


#if MODE_KAL == ACC_CONTROLLED

#define N_STATES 4
#define N_OBSERVABLES 2
#define N_FREQ 2
#define N_CONTROL_INPUT 2

#endif

#if MODE_KAL == ACC_CONTROLLED_UPG

#define N_STATES 6
#define N_OBSERVABLES 3
#define N_FREQ 2
#define N_CONTROL_INPUT 2

#endif

#if MODE_KAL == VEL_CONTROLLED

#define N_STATES 8		//x,y,heading,vx,vy,omega, acc_x,acc_y
#define N_OBSERVABLES 5 
#define N_FREQ 2
#define N_CONTROL_INPUT 2

#endif


#define F_GPS 1
#define F_ODOM 0



void kal_init_kalman(double state_0[N_STATES], double  cov_0[N_STATES][N_STATES],
			  double A[N_STATES][N_STATES], double B[N_STATES][N_CONTROL_INPUT],
			  double Q[N_STATES][N_STATES], double C[N_FREQ][N_OBSERVABLES][N_STATES],
			  double D[N_FREQ][N_OBSERVABLES][N_CONTROL_INPUT],double R[N_FREQ][N_OBSERVABLES][N_OBSERVABLES]);
void kal_predict(double u[N_CONTROL_INPUT]);
void kal_change_A(double new_A[N_STATES][N_STATES]);
void kal_change_B(double new_B[N_STATES][N_CONTROL_INPUT]);

void kal_update_freq(int freq, double y_meas[N_OBSERVABLES]);
void kal_get_pose(double pose[N_STATES]);
void kal_get_cov(double cov[N_STATES][N_STATES]);



#endif
