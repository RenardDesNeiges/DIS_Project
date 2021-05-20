#ifndef LINALG_H
#define LINALG_H
#include <stdbool.h>

void mat_T(int N, int M, double A[N][N], double At[N][N]);
void mat_comatrix(int N, double A[N][N], double C[N][N]);
void mat_add(int N, int M, double A[N][M], double B[N][M], double C[N][M]);
void mat_sub(int N, int M, double A[N][M], double B[N][M], double C[N][M]);

void mat_mult(int NA, int L, int MB, double A[NA][L], double B[L][MB], double C[NA][MB]);
bool mat_inv(int N, double A[N][N], double B[N][N]);
void vec_mult(int N, int M, double A[N][M], double v[M], double w[N]);
void vec_add(int N, double a[N], double b[N], double c[N]);
void vec_sub(int N, double a[N], double b[N], double c[N]);
double mat_det(int N, double A[N][N]);
void mat_decomp_LUP(int N, double A[N][N],double L[N][N], double U[N][N], double P[N][N]);
bool mat_inv_triangular(int N, double U[N][N], double invU[N][N]);
bool mat_permutation(int N, int M, int i, int j, double A[N][M]);
bool mat_inv_gauss(int N, double A[N][N], double invA[N][N]);

void mat_print(int N, int M, double A[N][M]);
void vec_print(int N, double v[N]);

#endif

