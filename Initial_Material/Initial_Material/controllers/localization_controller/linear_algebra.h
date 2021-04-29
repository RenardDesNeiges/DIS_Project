#ifndef LINALG_H
#define LINALG_H


void mat_T(int N, double A[N][N], double At[N][N]);
void mat_comatrix(int N, double A[N][N], double C[N][N]);
void mat_add(int N, int M, double A[N][M], double B[N][M], double C[N][M]);
void mat_mult(int NA, int L, int MB, double A[NA][L], double B[L][MB], double C[NA][MB]);
bool mat_inv(int N, double A[N][N], double B[N][N]);
void vec_mult(int N, int M, double A[N][M], double v[M], double w[N]);
double mat_det(int N, double A[N][N]);

#endif

