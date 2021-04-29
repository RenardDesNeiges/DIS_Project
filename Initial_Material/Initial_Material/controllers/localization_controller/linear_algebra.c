#include <stdio.h>
#include <stdbool.h>
#include "linear_algebra.h"
//-----------------------------------------------------------------------------------//
/*MACRO*/
#define CATCH(X,Y)      X = X || Y
#define CATCH_ERR(X,Y)  linalg_error(X, Y, __LINE__, __FILE__)


static bool linalg_error(bool test, const char * message, int line, const char * fileName);

/**
 * @brief Generic matrix multiplication function  C = A*B
 * @param[in] NA 	nb of lines of A (and C)
 * @param[in] L 	nb of lines of B and nb of column of A
 * @param[in] MB	nb of column of B (and C)
 * @param[in] A		array of size NAxMA
 * @param[in] B 	array of size NBxMB
 * @param[out]C		array of size NAxMB, C = A*B
 * 
 */
void mat_mult(int NA, int L, int MB, double A[NA][L], double B[L][MB], double C[NA][MB])
{
	
	for(int i = 0; i<NA;i++)
		for(int j = 0; j<MB;j++)
		{
			C[i][j] = 0;
			for(int k = 0; k<L;k++)
				C[i][j] += A[i][k]*B[k][j];
		}
}
/**
 * @brief Generic matrix addition function  C = A+B
 * @param[in] N 	nb of lines of A, B and C
 * @param[in] M		nb of column of A, B and C
 * @param[in] A		array of size NxM
 * @param[in] B 	array of size NxM
 * @param[out]C		array of size NxM, C = A+B
 * 
 */
void mat_add(int N, int M, double A[N][M], double B[N][M], double C[N][M])
{
	for(int i = 0; i<N;i++)
		for(int j = 0; j<M;j++)
				C[i][j] = A[i][j]+B[i][j];
}
/**
 * @brief Generic matrix multiplication of vector function (w = A*v)
 * @param[in] N 	nb of lines of A
 * @param[in] M		nb of column of A and nb of lines of v
 * @param[in] A		array of size NxM
 * @param[in] v 	array of size M
 * @param[out]w		array of size N, w = A*w
 * 
 */
void vec_mult(int N, int M, double A[N][M], double v[M], double w[N])
{
	for(int i = 0; i<M;i++)
	{
		w[i] = 0;
		for(int j = 0; j<N;j++)
				w[i] += A[j][i]+v[j];
	}
}
/**
 * @brief Generic matrix invers (B = inv(A)), use the Carley-Hamilton algorithm
 * @param[in] N 		nb of lines and columns of A and B
 * @param[in] A			array of size NxN
 * @param[out]B			invers of A
 * @return	  noninv	true if the matrix is not inversible
 * 
 */
bool mat_inv(int N, double A[N][N], double B[N][N])
{
	double C[N][N], Ct[N][N];
	double det;
	double tol = 0.0000001;
	det = mat_det(N,A);
	
	if (det < tol)
	{
		CATCH_ERR(det<tol,"matrix not inversible");
		return false;
	}
	mat_comatrix(N,A,C);
	mat_T(N,C,Ct);
	
	for(int i; i<N;i++)
		for(int j;j<N;j++)
			B[i][j] = Ct[i][j]/det;
	
	return true;
		
}
/**
 * @brief Generic matrix determinant, use the Carley-Hamilton algorithm
 * @param[in] N 	nb of lines and columns of A and B
 * @param[in] A		array of size NxN
 * @return 	  det	determinant of the matrix
 * 
 */
double mat_det(int N, double A[N][N])
{
	float sign = 1;
	double det = 0;
	double pivot[N-1][N-1];
	if (N==2)
	{
		det = A[0][0]*A[1][1]-A[1][0]*A[0][1];
	}
	else
	{
		
		for(int i = 0; i<N;i++)
			{
				if(A[i][0])
				{
					
					for(int k = 0; k<N-1;k++)
						for(int l = 0; l<N-1;l++)
						{
							if(k<i)
							{
								pivot[k][l] = A[k][l+1];
								continue;
							}
							if(k>=i)
							{
								pivot[k][l] = A[k+1][l+1];
								continue;
							}
						}
					det += sign*A[i][0]*mat_det(N-1,pivot);
					sign = -sign;
				}
			}
		
	}
	return det;
}
/**
 * @brief Generic comatrix calculation, use the Cramer rule
 * @param[in] N 	nb of lines and columns of A and B
 * @param[in] A		array of size NxN
 * @param[out]C 	Comatrix of A
 * 
 */
void mat_comatrix(int N, double A[N][N], double C[N][N])
{
	double cofactor[N-1][N-1];
	double sign = 1;
	for(int i=0; i<N;i++)
	{
		for(int j=0; j<N;j++)
		{
			for(int k = 0; k<N-1;k++)
				for(int l = 0; l<N-1;l++)
				{
					if(k<i && l>=j)
					{
						cofactor[k][l] = A[k][l+1];
						continue;
					}
					if(k>=i && l>=j)
					{
						cofactor[k][l] = A[k+1][l+1];
						continue;
					}
					if(k<i && l<j)
					{
						cofactor[k][l] = A[k][l];
						continue;
					}
					if(k>=i && l<j)
					{
						cofactor[k][l] = A[k+1][l];
						continue;
					}
				}
			printf("i,j: %i %i\n",i,j);
			printf("%f %f\n",cofactor[0][1],cofactor[0][1]);
			printf("%f %f\n",cofactor[1][0],cofactor[1][1]);
			C[i][j] = sign*mat_det(N-1,cofactor);
			sign = -sign;
		}
	}
}
/**
 * @brief Generic transpose calculation
 * @param[in] N 	nb of lines and columns of A and B
 * @param[in] A		array of size NxN
 * @param[out]At 	Comatrix of A
 * 
 */
void mat_T(int N, double A[N][N], double At[N][N])
{
	for(int i; i<N;i++)
		for(int j; j<N;j++)
			At[i][j] = A[j][i];
}
int main()
{
	double A[2][2] = {{1,1},{1,1}};
	double B[2][2] = {{1,1},{1,1}};
	double C[2][2];
	double D[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	double E[3][3]; 
	printf("A:\n");
	printf("%f %f\n",A[0][1],A[0][1]);
	printf("%f %f\n",A[1][0],A[1][1]);
	printf("B:\n");
	printf("%f %f\n",B[0][1],B[0][1]);
	printf("%f %f\n",B[1][0],B[1][1]);

	printf("C:\n");
	printf("%f %f\n",C[0][1],C[0][1]);
	printf("%f %f\n",C[1][0],C[1][1]);
	mat_mult(2,2,2,A,B,C);
	printf("C:\n");
	printf("%f %f\n",C[0][1],C[0][1]);
	printf("%f %f\n",C[1][0],C[1][1]);
	printf("A:\n");
	printf("%f %f\n",A[0][1],A[0][1]);
	printf("%f %f\n",A[1][0],A[1][1]);
	printf("D:\n");
	printf("%f %f %f\n",D[0][0],D[0][1],D[0][2]);
	printf("%f %f %f\n",D[1][0],D[1][1],D[1][2]);
	printf("%f %f %f\n",D[2][0],D[2][1],D[2][2]);
	printf("%f\n",mat_det(3,D));
	mat_comatrix(3,D,E);
	printf("E:\n");
	printf("%f %f %f\n",E[0][0],E[0][1],E[0][2]);
	printf("%f %f %f\n",E[1][0],E[1][1],E[1][2]);
	printf("%f %f %f\n",E[2][0],E[2][1],E[2][2]);
	printf("cul\n");
	return 1;
}


// -----------ERROR FUNCTION ----------
/**
 * @brief      Do an error test if the result is true write the message in the stderr.
 *
 * @param[in]  test     The error test to run
 * @param[in]  message  The error message
 *
 * @return     true if there is an error
 */
bool linalg_error(bool test, const char * message, int line, const char * fileName)
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

