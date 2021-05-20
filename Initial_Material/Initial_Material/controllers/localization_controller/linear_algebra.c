#include <stdio.h>
#include <math.h>
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
 * @brief Generic matrix addition function  C = A-B
 * @param[in] N 	nb of lines of A, B and C
 * @param[in] M		nb of column of A, B and C
 * @param[in] A		array of size NxM
 * @param[in] B 	array of size NxM
 * @param[out]C		array of size NxM, C = A-B
 * 
 */
void mat_sub(int N, int M, double A[N][M], double B[N][M], double C[N][M])
{
	for(int i = 0; i<N;i++)
		for(int j = 0; j<M;j++)
				C[i][j] = A[i][j]-B[i][j];
}
/**
 * @brief Generic matrix multiplication of vector function (w = A*v)
 * @param[in] N 	nb of lines of A
 * @param[in] M		nb of column of A and length of v
 * @param[in] A		array of size NxM
 * @param[in] v 	array of size M
 * @param[out]w		array of size N, w = A*v
 * 
 */
void vec_mult(int N, int M, double A[N][M], double v[M], double w[N])
{
	for(int i = 0; i<N;i++)
	{
		w[i] = 0;
		for(int j = 0; j<M;j++)
				w[i] += A[i][j]*v[j];
	}
}
/**
 * @brief Generic addition of vector function (c = a+b)
 * @param[in] N 	nb of dim of a, b and c
 * @param[in] a		array of size N
 * @param[in] b 	array of size N
 * @param[out]w		array of size N
 * 
 */
void vec_add(int N, double a[N], double b[N], double c[N])
{
	for(int i = 0; i<N;i++)
	{
		c[i] = a[i]+b[i];
	}
}
/**
 * @brief Generic substraction of vector function (c = a-b)
 * @param[in] N 	nb of dim of a, b and c
 * @param[in] a		array of size N
 * @param[in] b 	array of size N
 * @param[out]w		array of size N
 * 
 */
void vec_sub(int N, double a[N], double b[N], double c[N])
{
	for(int i = 0; i<N;i++)
	{
		c[i] = a[i]-b[i];
	}
}
/**
 * @brief Generic matrix invers (B = inv(A)), use the cramer's rule DEPRECATED
 * @param[in] N 		nb of lines and columns of A and B
 * @param[in] A			array of size NxN
 * @param[out]B			invers of A
 * @return	  noninv	true if the matrix is not inversible
 * 
 */
bool mat_inv_cramer(int N, double A[N][N], double B[N][N])
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
	mat_T(N,N,C,Ct);
	
	for(int i=0; i<N;i++)
		for(int j=0;j<N;j++)
			B[i][j] = Ct[i][j]/det;
	
	return true;
		
}
/**
 * @brief Generic matrix invers (B = inv(A)), use the Gauss formula
 * @param[in] N 		nb of lines and columns of A and B
 * @param[in] A			array of size NxN
 * @param[out]B			invers of A
 * @return	  noninv	true if the matrix is not inversible
 * 
 */
bool mat_inv_gauss(int N, double A[N][N], double invA[N][N])
{
	double P[N][N],L[N][N],U[N][N];
	double invU[N][N],invL[N][N], Lt[N][N], invLt[N][N];
	double invLU[N][N];
	mat_decomp_LUP(N,A,L,U,P);
	if(mat_inv_triangular(N,U,invU))
		return true;
		
	mat_T(N,N,L,Lt);
	mat_inv_triangular(N,Lt,invLt);
	mat_T(N,N,invLt,invL);
	mat_mult(N,N,N,invU,invL,invLU);
	mat_mult(N,N,N,invLU,P,invA);
	return false;
}

/**
 * @brief invert a definit upper triangular matrix
 * @param[in] N 		nb of lines and columns of A and B
 * @param[in] U			array of size NxN
 * @param[out]invU		invers of A
 * @return	  noninv	true if the matrix is not inversible
 * 
 */
bool mat_inv_triangular(int N, double U[N][N], double invU[N][N])
{
	//initialize invU
	for(int i = 0;i<N;i++)
		for(int j = 0;j<N;j++)
			invU[i][j] = (i==j)? 1:0;
	//check if inversible
	for(int i = 0;i<N;i++)
	{
		if(CATCH_ERR(U[i][i]==0,"matrix not invertible\n"))
			return true;
	}
	//for each column
	for(int i=N-1;i>=0;i--)
	{
		//normalize the main diagonal
		for(int j = i;j<N;j++)
		{
			invU[i][j] /= U[i][i];
		}
		//for each line over it
		for(int j=i-1;j>=0;j--)
		{
			
			//for each previous and current column
			for(int k = i;k<N;k++)
			{
				//subtract the line at every 
				invU[j][k] -= invU[i][k]*U[j][i];
				
			}
		}
	}
	
	return false;
}
/**
 * @brief 	find the P\A = LU decomposition of A, where L is a lower triangular matrix,
 * 			U is an upper triangular matrix and P is a permutation matrix
 * @param[in] N 		nb of lines and columns of A, L, U and P
 * @param[in] A			array of size NxN
 * @param[out] L		Lower triangular matrix
 * @param[out] U		Upper triangular matrix
 * @param[out] P		Permutation matrix
 * 
 */
void mat_decomp_LUP(int N, double A[N][N],double L[N][N], double U[N][N], double P[N][N])
{
	int maxidx;
	double buffer;
	//
	//initialize L and P (as the identity matrix) and U (as A)
	for(int i=0; i<N;i++)
		for(int j=0;j<N;j++)
		{
			L[i][j]=(i==j)? 1:0;
			P[i][j]=(i==j)? 1:0;
			U[i][j]=A[i][j];
		}
	for(int i = 0;i<N;i++)
	{
		
		//only consider the elements on or below the main diagonal
		maxidx = i;
		for(int j = i;j<N;j++)
			if (fabs(U[j][i])>fabs(U[maxidx][i]))
			{
				maxidx = j;
			}
		if (U[maxidx][i] == 0)
			return;
		if (maxidx != i)
		{
			
			mat_permutation(N,N,maxidx,i,P);
			mat_permutation(N,N,maxidx,i,U);
			//permute the previous elements in L
			for(int j = 0;j<i;j++)
			{
				buffer = L[maxidx][j];
				L[maxidx][j] = L[i][j];
				L[i][j] = buffer;
			}
		}
		for(int j  = i+1;j<N;j++) //each line below the pivot
		{
			L[j][i] = U[j][i]/U[i][i];
			for(int k = i;k<N;k++) //each colum after the pivot
				U[j][k] -= L[j][i]*U[i][k];
		}
	}
		
}
/**
 * @brief invert 2 line in a matrix and modify it
 * @param[in] N 	nb of lines  of A
 * @param[in] M 	nb of columns  of A
 * @param[in] i		first line to inverse
 * @param[in] j		second line to inverse
 * @param[in/out] A		array of size NxM
 * @return			true if error
 * 
 */
bool mat_permutation(int N, int M, int i, int j, double A[N][M])
{
	double buffer;
	if(CATCH_ERR((i>N || j>N),"indexes need to be smaller than N"))
		return true;
	for(int k = 0;k<M;k++)
	{
		buffer = A[i][k];
		A[i][k]=A[j][k];
		A[j][k]=buffer;
	}
	return false;
	
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
 * @param[in] N 	nb of lines of A
 * @param[in] M 	nb columns of A
 * @param[in] A		array of size NxM
 * @param[out]At 	Comatrix of A
 * 
 */
void mat_T(int N, int M, double A[N][M], double At[M][N])
{
	for(int i=0; i<N;i++)
		for(int j=0; j<M;j++)
		{
			At[j][i] = A[i][j];
		}
}
//~ int main()
//~ {
	//~ double A[4][4] = {{1,1,2,3},{1,1,4,6},{3,4,5,5},{3,5,6,7}};
	//~ double B[4][4] = {{1,1,2,3},{1,1,4,6},{3,4,5,5},{3,5,6,8}};
	//~ double C[4][4];
	//~ double v[4] ;
	//~ double w[2] = {0,1};
	//~ printf("A:\n");
	//~ mat_print(4,4,A);
	//~ printf("B:\n");
	//~ mat_print(4,4,B);
	//~ mat_sub(4,4,A,B,C);
	//~ printf("cul\n");
	//~ printf("C\n");
	//~ mat_print(4,4,C);
	//~ return 1;
//~ }
/**
 * @brief      printf the matrix
 *
 * @param[in]  N,M		format of the matrix
 * @param[in]  A  		NxM array representing the matrix
 *
 */
void mat_print(int N, int M, double A[N][M])
{
	for(int i = 0;i<N;i++)
	{
		for(int j = 0; j<M;j++)
			printf("%f ",A[i][j]);
		printf("\n");
	}
}
/**
 * @brief      printf the vector
 *
 * @param[in]  N		len of the vector
 * @param[in]  v  		N array representing the vector
 *
 */
void vec_print(int N, double v[N])
{
	for(int i = 0;i<N;i++)
	{
		printf("%f ",v[i]);
		printf("\n");
	}
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
    fprintf(stderr,"%s",buffer);

    return(true);
  }

  return false;
}

