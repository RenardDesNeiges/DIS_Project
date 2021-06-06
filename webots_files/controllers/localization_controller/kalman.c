/*
 * kalman.c
 * 
 * Copyright 2021 Flask <flask@Voiture>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include <stdio.h>
#include "kalman.h"
#include "linear_algebra.h"

//-----------------------------------------------------------------------------------//
/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  kal_error(X, Y, __LINE__, __FILE__)



// VERBOSE FLAG
#define VERBOSE_UPDATE false
#define VERBOSE_PREDICT false
//GLOBALS

//Model
//
/**Classical kalman model:
 * x(t+1) = A*x(t)+B*u(t) + v
 * y_i(t) = C_i*x(t)+D_i*u(t) + w_i
 * where v and w_i are random vectors, following a meanless normal law
 * Note that the observable y is separated into several measures y_i, 
 * in order to allow several different sample frequencies
 * 
 * cov_x(t+1) = A*cov_x(t)*At + Q
 * cov_y_i(t) = C_i*cov_x(t)*C_it +R_i
 * where Q is the covariance of v and R the covariance of w
 * Note that these covariances are the prior probabilistic distribution
 */
typedef struct 
{
	
	double x[N_STATES];
	double cov_x[N_STATES][N_STATES];
	double A[N_STATES][N_STATES];
	double B[N_STATES][N_CONTROL_INPUT];
	double Q[N_STATES][N_STATES];
	double y[N_FREQ][N_OBSERVABLES];
	double C[N_FREQ][N_OBSERVABLES][N_STATES];
	double D[N_FREQ][N_OBSERVABLES][N_CONTROL_INPUT];
	double R[N_FREQ][N_OBSERVABLES][N_OBSERVABLES];
} model_kalman_t;

/**Upgraded kalman model:
 * x(t+1) = A*x(t)+B*u(t)
 * y(t) = C*x(t)+D*u(t) + w
 * where x and u are independant random vectors following a normal law,
 * note that w was kept in order to describe the noise on measurement
 * 
 * cov_x(t+1) = A*cov_x(t)*At + B*cov_u(t)*Bt 
 * cov_y(t) = C*cov_x(t)*Ct + D*cov_u(t)*Dt + R
 * 
 * This model allows to have a variable cov_u(t) (for example 
 * by being proportional to the input u(t)), but requires to recompute
 * the kalman gain K at each iteration, thus requiring more computational
 * power
 */
typedef struct
{
	double x[N_STATES];
	double cov_u[N_CONTROL_INPUT][N_CONTROL_INPUT];
	double cov_x[N_STATES][N_STATES];
	double A[N_STATES][N_STATES];
	double B[N_STATES][N_CONTROL_INPUT];
	double y[N_OBSERVABLES];
	double C[N_FREQ][N_OBSERVABLES][N_STATES];
	double R[N_FREQ][N_OBSERVABLES][N_OBSERVABLES];
} model_rnd_vect_t;


//GLOBALS:
model_kalman_t kalman;
model_rnd_vect_t rnd_vec;



//functions

bool kal_error(bool test, const char * message, int line, const char * fileName);
static void kal_compute_K(int freq, double K[N_STATES][N_OBSERVABLES]);

/**
 * @brief	Initialize a kalman filter. This function allows to use custom
 * 			matrices to compute the variance of the model. To know the value 
 * 			and the meaning of the different dimentions, refer to the header file.
 * 			Note that in this kalman filter consider the input control as 
 * 			a random vector, thus the control input modifie the covariance of
 * 			both the state and the predicted observables.
 * 
 * 			
 * 			
 * 			pose_pred = A*state+B*control
 * 			cov_pred = A*cov*At + Q
 * 			y_pred = C*state+D*control
 * 			cov_y_pred = C*cov*Ct+R 
 * 
 * @param	state_0[N_STATES]			The initial state estimate of the robot
 * 										
 * 			cov_0[N_STATES][N_STATES]	The initial covariance on the position of the
 * 										robot. Can be set at 0 or at a low value
 * 
 * 			A[N_STATES][N_STATES]		Matrix describing the temporal change of the
 * 										pose
 * 			B[N_STATES][N_CONTROL]		Matrix describing the control result on the state
 * 			C[N_OBSERVABLE][N_STATES]	Matrix describing the observable from the state
 * 			D[N_OBSERVABLE][N_CONTROL]	Matrix describing the observable from the control
 * 			
 * 			Q[N_STATES][N_STATES]		Matrix describing the added covariance from the 
 * 										prediction (Q = B*cov_control*Bt) 
 * 		
 * 			
 * 			
 */
void kal_init_kalman(double state_0[N_STATES], double  cov_0[N_STATES][N_STATES],
			  double A[N_STATES][N_STATES], double B[N_STATES][N_CONTROL_INPUT],
			  double Q[N_STATES][N_STATES], double C[N_FREQ][N_OBSERVABLES][N_STATES],
			  double D[N_FREQ][N_OBSERVABLES][N_CONTROL_INPUT],double R[N_FREQ][N_OBSERVABLES][N_OBSERVABLES])
{
	
	//copy the different values
	for(int i=0;i<N_STATES;i++)
	{
		kalman.x[i] = state_0[i];
		for(int j = 0; j<N_STATES;j++)
		{
			kalman.cov_x[i][j] = cov_0[i][j];
			kalman.A[i][j] = A[i][j];
			kalman.Q[i][j] = Q[i][j];
		}
		for(int f = 0;f<N_FREQ;f++)
			for(int j = 0;j<N_OBSERVABLES;j++)
			{
				kalman.C[f][j][i] = C[f][j][i];
			}
	
		for(int j = 0; j<N_CONTROL_INPUT;j++)
		{
			kalman.B[i][j] = B[i][j];
		}
	}
	for(int f = 0; f<N_FREQ;f++)
		for(int i = 0;i<N_OBSERVABLES;i++)
		{
			for(int j =0;j<N_OBSERVABLES;j++)
				kalman.R[f][i][j] = R[f][i][j];
			for(int j =0;j<N_CONTROL_INPUT;j++)
				kalman.D[f][i][j]=D[f][i][j];
		}

}
/** 
 * @brief	Compute the optimal kalman gain of the measures done at a a 
 * 			frequency freq
 * @param[in] freq		The index of the matrix C and R to use
 * @param[out] K		The optimal gain
 **/
void kal_compute_K(int freq, double K[N_STATES][N_OBSERVABLES])
{
	double S[N_OBSERVABLES][N_OBSERVABLES],Ct[N_STATES][N_OBSERVABLES];
	double S_nonoise[N_OBSERVABLES][N_OBSERVABLES], covCt[N_STATES][N_OBSERVABLES];
	double invS[N_OBSERVABLES][N_OBSERVABLES];

	//compute S = C*cov_x*Ct+R
	mat_T(N_OBSERVABLES,N_STATES,kalman.C[freq],Ct);
	mat_mult(N_STATES,N_STATES,N_OBSERVABLES,kalman.cov_x,Ct,covCt);
	mat_mult(N_OBSERVABLES,N_STATES,N_OBSERVABLES,kalman.C[freq],covCt,S_nonoise);
	mat_add(N_OBSERVABLES, N_OBSERVABLES, S_nonoise, kalman.R[freq], S);

	
	//compute K = cov*Ct/S
	CATCH_ERR(mat_inv_gauss(N_OBSERVABLES,S,invS),"The S Matrix is not invertible\n");
	mat_mult(N_STATES,N_OBSERVABLES,N_OBSERVABLES,covCt,invS,K);
	
}
/** 
 * @brief				Update the matrix A in the model
 * @param[in] new_A		The new A matrix
 **/
void kal_change_A(double new_A[N_STATES][N_STATES])
{
	for(int i=0;i<N_STATES;i++)
		for(int j = 0; j<N_STATES;j++)
			kalman.A[i][j] = new_A[i][j];
}

/** 
 * @brief				Update the matrix B in the model
 * @param[in] new_B		The new B matrix
 **/
void kal_change_B(double new_B[N_STATES][N_CONTROL_INPUT])
{
	for(int i=0;i<N_STATES;i++)
		for(int j = 0; j<N_CONTROL_INPUT;j++)
			kalman.B[i][j] = new_B[i][j];
}
/** 
 * @brief				Update the matrix Q in the model
 * @param[in] new_Q		The new Q matrix
 **/
void kal_change_Q(double new_Q[N_STATES][N_STATES])
{
	for(int i=0;i<N_STATES;i++)
		for(int j = 0; j<N_STATES;j++)
			kalman.Q[i][j] = new_Q[i][j];
}

/**
 * @brief	Compute the prediction step of the kalman filter. The mode must be valid, and 
 * 			the number of inputs must be congruent
 * @param[in]	control		The set of control inputs; it needs to be initialised, 
 * 							congruently to the MODE used.
 **/
void kal_predict(double u[N_CONTROL_INPUT])
{
	double x_free[N_STATES],x_from_u[N_STATES];
	double At[N_STATES][N_STATES];
	double covAt[N_STATES][N_STATES];
	double noise_free_cov[N_STATES][N_STATES];
	
	//compute the free evolution of the states
	vec_mult(N_STATES,N_STATES,kalman.A,kalman.x,x_free);
	
	//compute the part caused by the control
	vec_mult(N_STATES,N_CONTROL_INPUT,kalman.B,u,x_from_u);
	//add the 2 parts
	vec_add(N_STATES,x_free,x_from_u,kalman.x);
	
	//compute the pred covariance
	mat_T(N_STATES,N_STATES,kalman.A,At);
	mat_mult(N_STATES,N_STATES,N_STATES,kalman.cov_x,At,covAt);
	mat_mult(N_STATES,N_STATES,N_STATES,kalman.A,covAt,noise_free_cov);
	mat_add(N_STATES,N_STATES,noise_free_cov,kalman.Q, kalman.cov_x);
	if(VERBOSE_PREDICT)
	{
		printf("state x (old): ");
		for(int i = 0;i<N_STATES;i++)
			printf("%f ",kalman.x[i]);
		printf("\n");
		printf("state x (from u): ");
		for(int i = 0;i<N_STATES;i++)
			printf("%e ",x_from_u[i]);
		printf("\n");
		printf("state x (free): ");
		for(int i = 0;i<N_STATES;i++)
			printf("%e ",x_free[i]);
		printf("\n");
	}
}
/**
 * @brief	Compute the update step of the kalman filter at a given frequency
 * @param[in]	freq		The frequency of interest
 * @param[in]	meas		The set of measurement;
 **/
void kal_update_freq(int freq, double y_meas[N_OBSERVABLES])
{
	double y_diff[N_OBSERVABLES],y_pred[N_OBSERVABLES];
	double new_x[N_STATES];
	double corr_x[N_STATES];
	double K[N_STATES][N_OBSERVABLES];
	double I[N_STATES][N_STATES];
	double gain_cov[N_STATES][N_STATES];
	double KC[N_STATES][N_STATES];
	double new_cov[N_STATES][N_STATES];
	
	//create an identity matrix of size N_STATES
	for(int i = 0;i<N_STATES;i++)
		for(int j= 0;j<N_STATES;j++)
			I[i][j] = (i==j)?1:0;
			
	kal_compute_K(freq,K);	
	
	//compute the difference
	vec_mult(N_OBSERVABLES,N_STATES,kalman.C[freq],kalman.x,y_pred);
	vec_sub(N_OBSERVABLES,y_meas,y_pred,y_diff);
	//update the estimate
	vec_mult(N_STATES,N_OBSERVABLES,K,y_diff,corr_x);
	if(VERBOSE_UPDATE)
	{
		printf("meas: ");
		for(int i = 0;i<N_OBSERVABLES;i++)
			printf("%f ",y_meas[i]);
		printf("\n");
		printf("pred: ");
		for(int i = 0;i<N_OBSERVABLES;i++)
			printf("%f ",y_pred[i]);
		printf("\n");
		printf("diff: ");
		for(int i = 0;i<N_OBSERVABLES;i++)
			printf("%f ",y_diff[i]);
		printf("\n");
	
		printf("corr x :");
		for(int i = 0;i<N_STATES;i++)
			printf("%f ",corr_x[i]);
		printf("\n");
	}
	vec_add(N_STATES,kalman.x,corr_x,new_x);
	

	//update the covariance
	

	mat_mult(N_STATES,N_OBSERVABLES,N_STATES,K,kalman.C[freq],KC);

	mat_sub(N_STATES,N_STATES, I,KC,gain_cov);
	
	mat_mult(N_STATES,N_STATES, N_STATES, gain_cov, kalman.cov_x,new_cov);
	//update in the struct
	for(int i = 0;i<N_STATES;i++)
	{
		kalman.x[i] = new_x[i];
		for(int j = 0;j<N_STATES;j++)
			kalman.cov_x[i][j] = new_cov[i][j];
	}
}


void kal_get_pose(double pose[N_STATES])
{
	for(int i = 0;i<N_STATES;i++)
	{
		pose[i] = kalman.x[i];
	}
}

bool kal_error(bool test, const char * message, int line, const char * fileName)
{
  if (test) 
  {
    char buffer[256];

    sprintf(buffer, "file : %s, line : %d,  error : %s", fileName, line, message);

    fprintf(stderr,"%s",buffer);

    return(true);
  }

  return false;
}

void kal_get_Q(double Q[N_STATES][N_STATES])
{
	for(int i =0; i<N_STATES;i++)
		for(int j=0; j<N_STATES;j++)
			Q[i][j] = kalman.Q[i][j];
}
