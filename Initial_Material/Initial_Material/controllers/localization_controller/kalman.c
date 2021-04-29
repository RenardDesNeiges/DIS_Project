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
#include "odometry.h"
#include "kalman.h"
#include "linear_algeabra.h"


//GLOBALS

const double R [4][4]
const double A[4][4]
const double B[4][2]
const  double C[2][4]
const double Q[2][2]
const double K [2][2]

//functions

static double* mat_mult(int NA, int MA, int NB, int MB, double * A, double * B);
bool kalman_error(bool test, const char * message, int line, const char * fileName);

void kal_init(measurement_t * meas,double[3]  pose, double[6][6]  cov ){
}
void kal_predict_acc(measurement_t * meas, double[3]  pose, double[6][6]  cov ){
}
void kal_update_enc(measurement_t * meas, double[3]  pose, double[6][6]  cov ){
}
void kal_update_gps(measurement_t * meas, double[3]  pose, double[6][6]  cov ){
}

static double* mat_mult(int NA, int MA, int NB, int MB, double * A, double * B){
	//sanitation check
	if (CATCH_ERR((MA != NB),"matrix dimention must agree"))
		return NULL;
	
	if (CATCH_ERR((sizeof A[0] / sizeof A[0][0] == NA
	
	{
		
}

bool kalman_error(bool test, const char * message, int line, const char * fileName)
{
  if (test) 
  {
    char buffer[256];

    sprintf(buffer, "file : %s, line : %d,  error : %s", fileName, line, message);

    fprintf(stderr,buffer);

    return(true);
  }

  return false;
}



