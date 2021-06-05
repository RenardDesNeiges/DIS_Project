/*
 * rungekutta.c
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
#include <math.h>
#include "rungekutta.h"


void fun(double y0[3], double t, double dy[3]);
int main(int argc, char **argv)
{
	double y0[3]={1,2,3}, yt[3];
	double dt = 1;
	double t = 0;
	euler_methode(3,y0,t,dt,&fun,yt);
	for(int i = 0;i<3;i++)
		printf("euler: %f \n",yt[i]); 
	runge_kutta_4(3,y0,t,dt,&fun,yt);
	for(int i = 0;i<3;i++)
		printf("runge: %f \n",yt[i]); 
	return 0;
}
void fun(double y0[3], double t0, double dy[3])
{
	dy[0] = 1;
	dy[1] = y0[1];
	dy[2] = y0[2]*t0;
}
/**
 * @brief		Compute the next point y using the euler method: y(n+1) = y(n)+dt*dy/dt(n)
 * @param[in]	dim		dimention of y
 * @param[in]	y0		initial y
 * @param[in]	t0		initial time
 * @param[in]	dt		time step
 * @param[in]	df		derivative of y by t: must be a function of the shape "void df(int dim, double t, double y0[dim])"
 * @param[out]	y1		y(n+1), the next point
 */
void euler_methode(int dim, double y0[dim], double t0, double dt, void (* df)(double[dim],double,double[dim]), double y1[dim])
{
	double dy[dim];
	df(y0,t0,dy);
	for(int i = 0;i<dim;i++)
	{
		y1[i] = y0[i]+dy[i]*dt;
	}
}

/**
 * @brief		Compute the next point y using the runge-kutta 4 method
 * @param[in]	dim		dimention of y
 * @param[in]	y0		initial y (can be called y(n))
 * @param[in]	t0		initial time
 * @param[in]	dt		time step
 * @param[in]	df		derivative of y by t: must be a function of the shape "void df(int dim, double t, double y0[dim])"
 * @param[out]	yf		y(n+1), the next point
 */
void runge_kutta_4(int dim, double y0[dim], double t0, double dt, void (* df)(double[dim],double,double[dim] ), double yf[dim])
{
	double k1[dim], k2[dim], k3[dim], k4[dim];
	double y1[dim], y2[dim], y3[dim];
	double tm = t0+dt/2;
	double tf = t0+dt;
	int i;
	
	df(y0,t0,k1);
	for(i=0;i<dim;i++)
		y1[i] = y0[i] + k1[i]*dt/2;
		
	df(y1,tm,k2);
	for(i=0;i<dim;i++)
		y2[i] = y0[i] + k2[i]*dt/2;
	
	df(y2,tm,k3);
	for(i=0;i<dim;i++)
		y3[i] = y0[i] + k3[i]*dt;
		
	df(y3,tf,k4);
	for(i=0; i<dim;i++)
		yf[i] = y0[i]+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
}


