#ifndef RUNGE_H
#define RUNGE_H




void euler_methode(int dim, double y0[dim], double t0, double dt, void (* df)(double[dim],double,double[dim] ), double y1[dim]);
void runge_kutta_4(int dim, double y0[dim], double t0, double dt, void (* df)(double[dim],double, double[dim] ), double yf[dim]);





#endif
