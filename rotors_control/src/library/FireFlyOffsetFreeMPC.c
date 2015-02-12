/*
FORCES - Fast interior point code generation for multistage problems.
Copyright (C) 2011-14 Alexander Domahidi [domahidi@control.ee.ethz.ch],
Automatic Control Laboratory, ETH Zurich.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "rotors_control/FireFlyOffsetFreeMPC.h"

/* for square root */
#include <math.h> 

/* SAFE DIVISION ------------------------------------------------------- */
#define MAX(X,Y)  ((X) < (Y) ? (Y) : (X))
#define MIN(X,Y)  ((X) < (Y) ? (X) : (Y))
/*#define SAFEDIV_POS(X,Y)  ( (Y) < EPS ? ((X)/EPS) : (X)/(Y) ) 
#define EPS (1.0000E-013) */
#define BIGM (1E30)
#define BIGMM (1E60)

/* includes for parallel computation if necessary */


/* SYSTEM INCLUDES FOR PRINTING ---------------------------------------- */


/* TIMING LIBRARY ------------------------------------------------- */

/* ARE WE ON WINDOWS? */
#if (defined WIN32 || defined _WIN64 || defined _WIN32)

/* Use Windows QueryPerformanceCounter for timing */

#include <windows.h>

typedef struct FireFlyOffsetFreeMPC_timer{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} FireFlyOffsetFreeMPC_timer;


void FireFlyOffsetFreeMPC_tic(FireFlyOffsetFreeMPC_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}



FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_toc(FireFlyOffsetFreeMPC_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (FireFlyOffsetFreeMPC_FLOAT)t->freq.QuadPart);
}


/* WE ARE ON THE MAC */
#elif (defined __APPLE__)
#include <mach/mach_time.h>


/* Use MAC OSX  mach_time for timing */
typedef struct FireFlyOffsetFreeMPC_timer{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;

} FireFlyOffsetFreeMPC_timer;


void FireFlyOffsetFreeMPC_tic(FireFlyOffsetFreeMPC_timer* t)
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}



FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_toc(FireFlyOffsetFreeMPC_timer* t)
{
    uint64_t duration; /* elapsed time in clock cycles*/
    t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (FireFlyOffsetFreeMPC_FLOAT)duration / 1000000000;
}

/* WE ARE ON SOME TEXAS INSTRUMENTS PLATFORM */
#elif (defined __TI_COMPILER_VERSION__)

/* TimeStamps */
#include <c6x.h> /* make use of TSCL, TSCH */


typedef struct FireFlyOffsetFreeMPC_timer{
	unsigned long long tic;
	unsigned long long toc;
} FireFlyOffsetFreeMPC_timer;


void FireFlyOffsetFreeMPC_tic(FireFlyOffsetFreeMPC_timer* t)
{
	TSCL = 0;	/* Initiate CPU timer by writing any val to TSCL */
	t->tic = _itoll( TSCH, TSCL );
}



FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_toc(FireFlyOffsetFreeMPC_timer* t)
{
	t->toc = _itoll( TSCH, TSCL );
	unsigned long long t0;
	unsigned long long overhead;
	t0 = _itoll( TSCH, TSCL );
	overhead = _itoll( TSCH, TSCL )  - t0;

	return (FireFlyOffsetFreeMPC_FLOAT)(t->toc - t->tic - overhead) / 1000000000;
}



/* WE ARE ON SOME OTHER UNIX/LINUX SYSTEM */
#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <time.h>
typedef struct FireFlyOffsetFreeMPC_timer{
	struct timespec tic;
	struct timespec toc;
} FireFlyOffsetFreeMPC_timer;


/* read current time */
void FireFlyOffsetFreeMPC_tic(FireFlyOffsetFreeMPC_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



/* return time passed since last call to tic on this timer */
double FireFlyOffsetFreeMPC_toc(FireFlyOffsetFreeMPC_timer* t)
{
	struct timespec temp;
	clock_gettime(CLOCK_MONOTONIC, &t->toc);	

	if ((t->toc.tv_nsec - t->tic.tv_nsec)<0) {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec-1;
		temp.tv_nsec = 1000000000+t->toc.tv_nsec - t->tic.tv_nsec;
	} else {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_nsec = t->toc.tv_nsec - t->tic.tv_nsec;
	}

	return (FireFlyOffsetFreeMPC_FLOAT)temp.tv_sec + (FireFlyOffsetFreeMPC_FLOAT)temp.tv_nsec / 1000000000;
}


#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 337 with a value.
 */
void FireFlyOffsetFreeMPC_LA_INITIALIZEVECTOR_337(FireFlyOffsetFreeMPC_FLOAT* vec, FireFlyOffsetFreeMPC_FLOAT value)
{
	int i;
	for( i=0; i<337; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 280 with a value.
 */
void FireFlyOffsetFreeMPC_LA_INITIALIZEVECTOR_280(FireFlyOffsetFreeMPC_FLOAT* vec, FireFlyOffsetFreeMPC_FLOAT value)
{
	int i;
	for( i=0; i<280; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 234 with a value.
 */
void FireFlyOffsetFreeMPC_LA_INITIALIZEVECTOR_234(FireFlyOffsetFreeMPC_FLOAT* vec, FireFlyOffsetFreeMPC_FLOAT value)
{
	int i;
	for( i=0; i<234; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 234.
 */
void FireFlyOffsetFreeMPC_LA_DOTACC_234(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<234; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, dense matrix of size [17 x 17]
 *             f  - column vector of size 17
 *             z  - column vector of size 17
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 17
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(FireFlyOffsetFreeMPC_FLOAT* H, FireFlyOffsetFreeMPC_FLOAT* f, FireFlyOffsetFreeMPC_FLOAT* z, FireFlyOffsetFreeMPC_FLOAT* grad, FireFlyOffsetFreeMPC_FLOAT* value)
{
	int i;
	int j;
	int k = 0;
	FireFlyOffsetFreeMPC_FLOAT hz;	
	for( i=0; i<17; i++){
		hz = 0;
		for( j=0; j<17; j++ )
		{
			hz += H[k++]*z[j];
		}
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, dense matrix of size [14 x 14]
 *             f  - column vector of size 14
 *             z  - column vector of size 14
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 14
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_14(FireFlyOffsetFreeMPC_FLOAT* H, FireFlyOffsetFreeMPC_FLOAT* f, FireFlyOffsetFreeMPC_FLOAT* z, FireFlyOffsetFreeMPC_FLOAT* grad, FireFlyOffsetFreeMPC_FLOAT* value)
{
	int i;
	int j;
	int k = 0;
	FireFlyOffsetFreeMPC_FLOAT hz;	
	for( i=0; i<14; i++){
		hz = 0;
		for( j=0; j<14; j++ )
		{
			hz += H[k++]*z[j];
		}
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB3_28_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *l, FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *z, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	FireFlyOffsetFreeMPC_FLOAT AxBu[28];
	FireFlyOffsetFreeMPC_FLOAT norm = *y;
	FireFlyOffsetFreeMPC_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<28; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<28; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
		for( i=0; i<28; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<28; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *l, FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *z, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	FireFlyOffsetFreeMPC_FLOAT AxBu[14];
	FireFlyOffsetFreeMPC_FLOAT norm = *y;
	FireFlyOffsetFreeMPC_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<14; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<17; j++ ){		
		for( i=0; i<14; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<14; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *l, FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *z, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	FireFlyOffsetFreeMPC_FLOAT AxBu[14];
	FireFlyOffsetFreeMPC_FLOAT norm = *y;
	FireFlyOffsetFreeMPC_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<14; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<17; j++ ){		
		for( i=0; i<14; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<14; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [28 x 17]
 * and stored in column major format. Note the transpose of M!
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MTVM_28_17(FireFlyOffsetFreeMPC_FLOAT *M, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<17; i++ ){
		y[i] = 0;
		for( j=0; j<28; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [14 x 17]
 * and B is of size [28 x 17]
 * and stored in column major format. Note the transposes of A and B!
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MTVM2_14_17_28(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<17; i++ ){
		z[i] = 0;
		for( j=0; j<14; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<28; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [14 x 17] and stored in column major format.
 * and B is of size [14 x 17] and stored in diagzero format
 * Note the transposes of A and B!
 */
void FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	for( i=0; i<14; i++ ){
		z[i] = 0;
		for( j=0; j<14; j++ ){
			z[i] += A[k++]*x[j];
		}
		z[i] += B[i]*y[i];
	}
	for( i=14 ;i<17; i++ ){
		z[i] = 0;
		for( j=0; j<14; j++ ){
			z[i] += A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [14 x 14]
 * and stored in diagzero format. Note the transpose of M!
 */
void FireFlyOffsetFreeMPC_LA_DIAGZERO_MTVM_14_14(FireFlyOffsetFreeMPC_FLOAT *M, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	for( i=0; i<14; i++ ){
		y[i] = M[i]*x[i];
	}
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 6. Output z is of course scalar.
 */
void FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_FLOAT* t, FireFlyOffsetFreeMPC_FLOAT* u, int* uidx, FireFlyOffsetFreeMPC_FLOAT* v, FireFlyOffsetFreeMPC_FLOAT* w, FireFlyOffsetFreeMPC_FLOAT* y, FireFlyOffsetFreeMPC_FLOAT* z, FireFlyOffsetFreeMPC_FLOAT* r)
{
	int i;
	FireFlyOffsetFreeMPC_FLOAT norm = *r;
	FireFlyOffsetFreeMPC_FLOAT vx = 0;
	FireFlyOffsetFreeMPC_FLOAT x;
	for( i=0; i<6; i++){
		x = t[i] - u[uidx[i]];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 6. Output z is of course scalar.
 */
void FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_FLOAT* t, int* tidx, FireFlyOffsetFreeMPC_FLOAT* u, FireFlyOffsetFreeMPC_FLOAT* v, FireFlyOffsetFreeMPC_FLOAT* w, FireFlyOffsetFreeMPC_FLOAT* y, FireFlyOffsetFreeMPC_FLOAT* z, FireFlyOffsetFreeMPC_FLOAT* r)
{
	int i;
	FireFlyOffsetFreeMPC_FLOAT norm = *r;
	FireFlyOffsetFreeMPC_FLOAT vx = 0;
	FireFlyOffsetFreeMPC_FLOAT x;
	for( i=0; i<6; i++){
		x = t[tidx[i]] - u[i];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 3. Output z is of course scalar.
 */
void FireFlyOffsetFreeMPC_LA_VSUBADD3_3(FireFlyOffsetFreeMPC_FLOAT* t, FireFlyOffsetFreeMPC_FLOAT* u, int* uidx, FireFlyOffsetFreeMPC_FLOAT* v, FireFlyOffsetFreeMPC_FLOAT* w, FireFlyOffsetFreeMPC_FLOAT* y, FireFlyOffsetFreeMPC_FLOAT* z, FireFlyOffsetFreeMPC_FLOAT* r)
{
	int i;
	FireFlyOffsetFreeMPC_FLOAT norm = *r;
	FireFlyOffsetFreeMPC_FLOAT vx = 0;
	FireFlyOffsetFreeMPC_FLOAT x;
	for( i=0; i<3; i++){
		x = t[i] - u[uidx[i]];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 3. Output z is of course scalar.
 */
void FireFlyOffsetFreeMPC_LA_VSUBADD2_3(FireFlyOffsetFreeMPC_FLOAT* t, int* tidx, FireFlyOffsetFreeMPC_FLOAT* u, FireFlyOffsetFreeMPC_FLOAT* v, FireFlyOffsetFreeMPC_FLOAT* w, FireFlyOffsetFreeMPC_FLOAT* y, FireFlyOffsetFreeMPC_FLOAT* z, FireFlyOffsetFreeMPC_FLOAT* r)
{
	int i;
	FireFlyOffsetFreeMPC_FLOAT norm = *r;
	FireFlyOffsetFreeMPC_FLOAT vx = 0;
	FireFlyOffsetFreeMPC_FLOAT x;
	for( i=0; i<3; i++){
		x = t[tidx[i]] - u[i];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 17
 * Returns also L/S, a value that is often used elsewhere.
 */
void FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_FLOAT *lu, FireFlyOffsetFreeMPC_FLOAT *su, FireFlyOffsetFreeMPC_FLOAT *ru, FireFlyOffsetFreeMPC_FLOAT *ll, FireFlyOffsetFreeMPC_FLOAT *sl, FireFlyOffsetFreeMPC_FLOAT *rl, int* lbIdx, int* ubIdx, FireFlyOffsetFreeMPC_FLOAT *grad, FireFlyOffsetFreeMPC_FLOAT *lubysu, FireFlyOffsetFreeMPC_FLOAT *llbysl)
{
	int i;
	for( i=0; i<17; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<6; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<6; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 14
 * Returns also L/S, a value that is often used elsewhere.
 */
void FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_14_3_3(FireFlyOffsetFreeMPC_FLOAT *lu, FireFlyOffsetFreeMPC_FLOAT *su, FireFlyOffsetFreeMPC_FLOAT *ru, FireFlyOffsetFreeMPC_FLOAT *ll, FireFlyOffsetFreeMPC_FLOAT *sl, FireFlyOffsetFreeMPC_FLOAT *rl, int* lbIdx, int* ubIdx, FireFlyOffsetFreeMPC_FLOAT *grad, FireFlyOffsetFreeMPC_FLOAT *lubysu, FireFlyOffsetFreeMPC_FLOAT *llbysl)
{
	int i;
	for( i=0; i<14; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<3; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<3; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 337.
 */
void FireFlyOffsetFreeMPC_LA_VVADD3_337(FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *v, FireFlyOffsetFreeMPC_FLOAT *w, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<337; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 17.
 *
 * Inputs: - H = dense cost Hessian in column major storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(FireFlyOffsetFreeMPC_FLOAT *H, FireFlyOffsetFreeMPC_FLOAT *llbysl, int* lbIdx, FireFlyOffsetFreeMPC_FLOAT *lubysu, int* ubIdx, FireFlyOffsetFreeMPC_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy lower triangular part of H into PHI */
	for( i=0; i<17; i++ ){
		for( j=0; j<=i; j++ ){
			Phi[k++] = H[i*17+j];
		}		
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<6; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<6; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 17.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    FireFlyOffsetFreeMPC_FLOAT l;
    FireFlyOffsetFreeMPC_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<17; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if FireFlyOffsetFreeMPC_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL2): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 A[ii+i] = 2.0000000000000000E-002;
		} else
		{
			A[ii+i] = sqrt(Mii);
		}
#else
		A[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif
                    
		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<17; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += A[jj+k]*A[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            A[jj+i] = (A[jj+i] - l)/A[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [28 x 17],
 * B is given and of size [28 x 17], L is a lower tri-
 * angular matrix of size 17 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_28_17(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    FireFlyOffsetFreeMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<17; j++ ){        
        for( i=0; i<28; i++ ){
            a = B[j*28+i];
            for( k=0; k<j; k++ ){
                a -= A[k*28+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*28+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 17.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *y)
{
    int i,j,ii,di;
    FireFlyOffsetFreeMPC_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<17; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability  */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM);

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [14 x 17],
 * B is given and of size [14 x 17], L is a lower tri-
 * angular matrix of size 17 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    FireFlyOffsetFreeMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<17; j++ ){        
        for( i=0; i<14; i++ ){
            a = B[j*14+i];
            for( k=0; k<j; k++ ){
                a -= A[k*14+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*14+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [28 x 17]
 *  size(B) = [14 x 17]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MMTM_28_17_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *C)
{
    int i, j, k;
    FireFlyOffsetFreeMPC_FLOAT temp;
    
    for( i=0; i<28; i++ ){        
        for( j=0; j<14; j++ ){
            temp = 0; 
            for( k=0; k<17; k++ ){
                temp += A[k*28+i]*B[k*14+j];
            }						
            C[j*28+i] = temp;
        }
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [14 x 17],
 * B is given and of size [14 x 17] stored in 
 * diagzero storage format, L is a lower tri-
 * angular matrix of size 17 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    FireFlyOffsetFreeMPC_FLOAT a;
	
	/*
	* The matrix A has the form
	*
	* d u u u r r r r r 
	* 0 d u u r r r r r 
	* 0 0 d u r r r r r 
	* 0 0 0 d r r r r r
	*
	* |Part1|| Part 2 |
	* 
	* d: diagonal
	* u: upper
	* r: right
	*/
	
	
    /* Part 1 */
    ii=0; di=0;
    for( j=0; j<14; j++ ){        
        for( i=0; i<j; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "u"
             * i < j */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*14+i]*L[ii+k];
            }
            A[j*14+i] = a/L[ii+j];
        }
        /* do the diagonal "d"
         * i = j */
        A[j*14+j] = B[i]/L[ii+j];
        
        /* fill lower triangular part with zeros "0"
         * n > i > j */
        for( i=j+1     ; i < 14; i++ ){
            A[j*14+i] = 0;
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	/* Part 2 */ 
	for( j=14; j<17; j++ ){        
        for( i=0; i<14; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "r" */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*14+i]*L[ii+k];
            }
            A[j*14+i] = a/L[ii+j];
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	
	
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [14 x 17]
 *  size(B) = [14 x 17]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *C)
{
    int i, j, k;
    FireFlyOffsetFreeMPC_FLOAT temp;
    
    for( i=0; i<14; i++ ){        
        for( j=0; j<14; j++ ){
            temp = 0; 
            for( k=0; k<17; k++ ){
                temp += A[k*14+i]*B[k*14+j];
            }						
            C[j*14+i] = temp;
        }
    }
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 14.
 *
 * Inputs: - H = dense cost Hessian in column major storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_14_3_3(FireFlyOffsetFreeMPC_FLOAT *H, FireFlyOffsetFreeMPC_FLOAT *llbysl, int* lbIdx, FireFlyOffsetFreeMPC_FLOAT *lubysu, int* ubIdx, FireFlyOffsetFreeMPC_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy lower triangular part of H into PHI */
	for( i=0; i<14; i++ ){
		for( j=0; j<=i; j++ ){
			Phi[k++] = H[i*14+j];
		}		
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<3; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<3; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 14.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_14(FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    FireFlyOffsetFreeMPC_FLOAT l;
    FireFlyOffsetFreeMPC_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<14; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if FireFlyOffsetFreeMPC_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL2): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 A[ii+i] = 2.0000000000000000E-002;
		} else
		{
			A[ii+i] = sqrt(Mii);
		}
#else
		A[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif
                    
		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<14; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += A[jj+k]*A[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            A[jj+i] = (A[jj+i] - l)/A[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [14 x 14],
 * B is given and of size [14 x 14] stored in 
 * diagzero storage format, L is a lower tri-
 * angular matrix of size 14 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_14(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    FireFlyOffsetFreeMPC_FLOAT a;
	
	/*
	* The matrix A has the form
	*
	* d u u u r r r r r 
	* 0 d u u r r r r r 
	* 0 0 d u r r r r r 
	* 0 0 0 d r r r r r
	*
	* |Part1|| Part 2 |
	* 
	* d: diagonal
	* u: upper
	* r: right
	*/
	
	
    /* Part 1 */
    ii=0; di=0;
    for( j=0; j<14; j++ ){        
        for( i=0; i<j; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "u"
             * i < j */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*14+i]*L[ii+k];
            }
            A[j*14+i] = a/L[ii+j];
        }
        /* do the diagonal "d"
         * i = j */
        A[j*14+j] = B[i]/L[ii+j];
        
        /* fill lower triangular part with zeros "0"
         * n > i > j */
        for( i=j+1     ; i < 14; i++ ){
            A[j*14+i] = 0;
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	/* Part 2 */ 
	for( j=14; j<14; j++ ){        
        for( i=0; i<14; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "r" */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*14+i]*L[ii+k];
            }
            A[j*14+i] = a/L[ii+j];
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	
	
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 14.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *y)
{
    int i,j,ii,di;
    FireFlyOffsetFreeMPC_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<14; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability  */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM);

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [28 x 17] in column
 * storage format, and B is of size [28 x 17] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MMT2_28_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    FireFlyOffsetFreeMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<28; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<17; k++ ){
                ltemp += A[k*28+i]*A[k*28+j];
            }			
			for( k=0; k<17; k++ ){
                ltemp += B[k*28+i]*B[k*28+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_28_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<28; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<28; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
		for( i=0; i<28; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [14 x 17] in column
 * storage format, and B is of size [14 x 17] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    FireFlyOffsetFreeMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<14; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<17; k++ ){
                ltemp += A[k*14+i]*A[k*14+j];
            }			
			for( k=0; k<17; k++ ){
                ltemp += B[k*14+i]*B[k*14+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<14; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<14; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
		for( i=0; i<14; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [14 x 17] in column
 * storage format, and B is of size [14 x 14] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    FireFlyOffsetFreeMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<14; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<17; k++ ){
                ltemp += A[k*14+i]*A[k*14+j];
            }			
			for( k=0; k<14; k++ ){
                ltemp += B[k*14+i]*B[k*14+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<14; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<14; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
		for( i=0; i<14; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 28 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_CHOL_28(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    FireFlyOffsetFreeMPC_FLOAT l;
    FireFlyOffsetFreeMPC_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<28; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<28; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
#if FireFlyOffsetFreeMPC_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 L[ii+i] = 2.0000000000000000E-002;
		} else
		{
			L[ii+i] = sqrt(Mii);
		}
#else
		L[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<28; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += L[jj+k]*L[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            L[jj+i] = (L[jj+i] - l)/L[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }	
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 28.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_28(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *y)
{
    int i,j,ii,di;
    FireFlyOffsetFreeMPC_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<28; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability  */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM);

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/** 
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [14 x 28],
 * B is given and of size [14 x 28], L is a lower tri-
 * angular matrix of size 28 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_28(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i,j,k,ii,di;
    FireFlyOffsetFreeMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<28; j++ ){        
        for( i=0; i<14; i++ ){
            a = B[i*28+j];
            for( k=0; k<j; k++ ){
                a -= A[k*14+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*14+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 14
 * and A is a dense matrix of size [14 x 28] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_28(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    FireFlyOffsetFreeMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<14; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<28; k++ ){
                ltemp += A[k*14+i]*A[k*14+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 14 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    FireFlyOffsetFreeMPC_FLOAT l;
    FireFlyOffsetFreeMPC_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<14; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<14; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
#if FireFlyOffsetFreeMPC_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 L[ii+i] = 2.0000000000000000E-002;
		} else
		{
			L[ii+i] = sqrt(Mii);
		}
#else
		L[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<14; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += L[jj+k]*L[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            L[jj+i] = (L[jj+i] - l)/L[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }	
}


/* 
 * Computes r = b - A*x
 * where A is stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_28(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<14; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<28; j++ ){		
		for( i=0; i<14; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/** 
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [14 x 14],
 * B is given and of size [14 x 14], L is a lower tri-
 * angular matrix of size 14 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *A)
{
    int i,j,k,ii,di;
    FireFlyOffsetFreeMPC_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<14; j++ ){        
        for( i=0; i<14; i++ ){
            a = B[i*14+j];
            for( k=0; k<j; k++ ){
                a -= A[k*14+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*14+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 14
 * and A is a dense matrix of size [14 x 14] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *L)
{
    int i, j, k, ii, di;
    FireFlyOffsetFreeMPC_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<14; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<14; k++ ){
                ltemp += A[k*14+i]*A[k*14+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x
 * where A is stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<14; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<14; j++ ){		
		for( i=0; i<14; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 14.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    FireFlyOffsetFreeMPC_FLOAT xel;    
	int start = 91;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 13;
    for( i=13; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 13;
        for( j=13; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }

		/* saturate for numerical stability */
		xel = MIN(xel, BIGM);
		xel = MAX(xel, -BIGM); 

        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [14 x 14]
 * and stored in column major format. Note the transpose of M!
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<14; i++ ){
		r[i] = b[i];
		for( j=0; j<14; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [14 x 28]
 * and stored in column major format. Note the transpose of M!
 */
void FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_28(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<28; i++ ){
		r[i] = b[i];
		for( j=0; j<14; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 28.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_28(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    FireFlyOffsetFreeMPC_FLOAT xel;    
	int start = 378;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 27;
    for( i=27; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 27;
        for( j=27; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }

		/* saturate for numerical stability */
		xel = MIN(xel, BIGM);
		xel = MAX(xel, -BIGM); 

        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Vector subtraction z = -x - y for vectors of length 337.
 */
void FireFlyOffsetFreeMPC_LA_VSUB2_337(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<337; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 17 in lower triangular
 * storage format.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    FireFlyOffsetFreeMPC_FLOAT y[17];
    FireFlyOffsetFreeMPC_FLOAT yel,xel;
	int start = 136;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<17; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM); 

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 16;
    for( i=16; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 16;
        for( j=16; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }

		/* saturate for numerical stability */
		xel = MIN(xel, BIGM);
		xel = MAX(xel, -BIGM); 

        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 14 in lower triangular
 * storage format.
 */
void FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_14(FireFlyOffsetFreeMPC_FLOAT *L, FireFlyOffsetFreeMPC_FLOAT *b, FireFlyOffsetFreeMPC_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    FireFlyOffsetFreeMPC_FLOAT y[14];
    FireFlyOffsetFreeMPC_FLOAT yel,xel;
	int start = 91;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<14; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM); 

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 13;
    for( i=13; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 13;
        for( j=13; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }

		/* saturate for numerical stability */
		xel = MIN(xel, BIGM);
		xel = MAX(xel, -BIGM); 

        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 6,
 * and x has length 17 and is indexed through yidx.
 */
void FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_FLOAT *x, int* xidx, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 6.
 */
void FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *v, FireFlyOffsetFreeMPC_FLOAT *w, FireFlyOffsetFreeMPC_FLOAT *x)
{
	int i;
	for( i=0; i<6; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 17
 * and z, x and yidx are of length 6.
 */
void FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y, int* yidx, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 3,
 * and x has length 14 and is indexed through yidx.
 */
void FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_3(FireFlyOffsetFreeMPC_FLOAT *x, int* xidx, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<3; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 3.
 */
void FireFlyOffsetFreeMPC_LA_VSUB3_3(FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *v, FireFlyOffsetFreeMPC_FLOAT *w, FireFlyOffsetFreeMPC_FLOAT *x)
{
	int i;
	for( i=0; i<3; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 14
 * and z, x and yidx are of length 3.
 */
void FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_3(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y, int* yidx, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<3; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/**
 * Backtracking line search.
 * 
 * First determine the maximum line length by a feasibility line
 * search, i.e. a ~= argmax{ a \in [0...1] s.t. l+a*dl >= 0 and s+a*ds >= 0}.
 *
 * The function returns either the number of iterations or exits the error code
 * FireFlyOffsetFreeMPC_NOPROGRESS (should be negative).
 */
int FireFlyOffsetFreeMPC_LINESEARCH_BACKTRACKING_AFFINE(FireFlyOffsetFreeMPC_FLOAT *l, FireFlyOffsetFreeMPC_FLOAT *s, FireFlyOffsetFreeMPC_FLOAT *dl, FireFlyOffsetFreeMPC_FLOAT *ds, FireFlyOffsetFreeMPC_FLOAT *a, FireFlyOffsetFreeMPC_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    FireFlyOffsetFreeMPC_FLOAT dltemp;
    FireFlyOffsetFreeMPC_FLOAT dstemp;
    FireFlyOffsetFreeMPC_FLOAT mya = 1.0;
    FireFlyOffsetFreeMPC_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<234; i++ ){
            dltemp = l[i] + mya*dl[i];
            dstemp = s[i] + mya*ds[i];
            if( dltemp < 0 || dstemp < 0 ){
                lsIt++;
                break;
            } else {                
                mymu += dstemp*dltemp;
            }
        }
        
        /* 
         * If no early termination of the for-loop above occurred, we
         * found the required value of a and we can quit the while loop.
         */
        if( i == 234 ){
            break;
        } else {
            mya *= FireFlyOffsetFreeMPC_SET_LS_SCALE_AFF;
            if( mya < FireFlyOffsetFreeMPC_SET_LS_MINSTEP ){
                return FireFlyOffsetFreeMPC_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (FireFlyOffsetFreeMPC_FLOAT)234;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 234.
 */
void FireFlyOffsetFreeMPC_LA_VSUB5_234(FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *v, FireFlyOffsetFreeMPC_FLOAT mu,  FireFlyOffsetFreeMPC_FLOAT sigma, FireFlyOffsetFreeMPC_FLOAT *x)
{
	int i;
	for( i=0; i<234; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 17,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *su, int* uidx, FireFlyOffsetFreeMPC_FLOAT *v, FireFlyOffsetFreeMPC_FLOAT *sv, int* vidx, FireFlyOffsetFreeMPC_FLOAT *x)
{
	int i;
	for( i=0; i<17; i++ ){
		x[i] = 0;
	}
	for( i=0; i<6; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<6; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_28_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<28; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<17; j++ ){		
		for( i=0; i<28; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
		for( i=0; i<28; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<14; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<17; j++ ){		
		for( i=0; i<14; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
		for( i=0; i<14; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 14,
 * u, su, uidx are of length 3 and v, sv, vidx are of length 3.
 */
void FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_14_3_3(FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *su, int* uidx, FireFlyOffsetFreeMPC_FLOAT *v, FireFlyOffsetFreeMPC_FLOAT *sv, int* vidx, FireFlyOffsetFreeMPC_FLOAT *x)
{
	int i;
	for( i=0; i<14; i++ ){
		x[i] = 0;
	}
	for( i=0; i<3; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<3; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_14(FireFlyOffsetFreeMPC_FLOAT *A, FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *B, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<14; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<17; j++ ){		
		for( i=0; i<14; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
		for( i=0; i<14; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Vector subtraction z = x - y for vectors of length 337.
 */
void FireFlyOffsetFreeMPC_LA_VSUB_337(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<337; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *s, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *y, int* yidx, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *s, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *y, int* yidx, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 3 (length of y >= 3).
 */
void FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_3(FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *s, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *y, int* yidx, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<3; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 3 (length of y >= 3).
 */
void FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_3(FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *s, FireFlyOffsetFreeMPC_FLOAT *u, FireFlyOffsetFreeMPC_FLOAT *y, int* yidx, FireFlyOffsetFreeMPC_FLOAT *z)
{
	int i;
	for( i=0; i<3; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 234.
 */
void FireFlyOffsetFreeMPC_LA_VSUB7_234(FireFlyOffsetFreeMPC_FLOAT *l, FireFlyOffsetFreeMPC_FLOAT *r, FireFlyOffsetFreeMPC_FLOAT *s, FireFlyOffsetFreeMPC_FLOAT *dl, FireFlyOffsetFreeMPC_FLOAT *ds)
{
	int i;
	for( i=0; i<234; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 337.
 */
void FireFlyOffsetFreeMPC_LA_VADD_337(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	for( i=0; i<337; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 280.
 */
void FireFlyOffsetFreeMPC_LA_VADD_280(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	for( i=0; i<280; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 234.
 */
void FireFlyOffsetFreeMPC_LA_VADD_234(FireFlyOffsetFreeMPC_FLOAT *x, FireFlyOffsetFreeMPC_FLOAT *y)
{
	int i;
	for( i=0; i<234; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int FireFlyOffsetFreeMPC_LINESEARCH_BACKTRACKING_COMBINED(FireFlyOffsetFreeMPC_FLOAT *z, FireFlyOffsetFreeMPC_FLOAT *v, FireFlyOffsetFreeMPC_FLOAT *l, FireFlyOffsetFreeMPC_FLOAT *s, FireFlyOffsetFreeMPC_FLOAT *dz, FireFlyOffsetFreeMPC_FLOAT *dv, FireFlyOffsetFreeMPC_FLOAT *dl, FireFlyOffsetFreeMPC_FLOAT *ds, FireFlyOffsetFreeMPC_FLOAT *a, FireFlyOffsetFreeMPC_FLOAT *mu)
{
    int i, lsIt=1;       
    FireFlyOffsetFreeMPC_FLOAT dltemp;
    FireFlyOffsetFreeMPC_FLOAT dstemp;    
    FireFlyOffsetFreeMPC_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<234; i++ ){
            dltemp = l[i] + (*a)*dl[i];
            dstemp = s[i] + (*a)*ds[i];
            if( dltemp < 0 || dstemp < 0 ){
                lsIt++;
                break;
            }
        }
        
        /* 
         * If no early termination of the for-loop above occurred, we
         * found the required value of a and we can quit the while loop.
         */
        if( i == 234 ){
            break;
        } else {
            *a *= FireFlyOffsetFreeMPC_SET_LS_SCALE;
            if( *a < FireFlyOffsetFreeMPC_SET_LS_MINSTEP ){
                return FireFlyOffsetFreeMPC_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*FireFlyOffsetFreeMPC_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<337; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<280; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<234; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (FireFlyOffsetFreeMPC_FLOAT)234;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_z[337];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_v[280];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_dz_aff[337];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_dv_aff[280];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_grad_cost[337];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_grad_eq[337];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rd[337];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_l[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_s[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lbys[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_dl_aff[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ds_aff[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_dz_cc[337];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_dv_cc[280];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_dl_cc[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ds_cc[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ccrhs[234];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_grad_ineq[337];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z00 = FireFlyOffsetFreeMPC_z + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff00 = FireFlyOffsetFreeMPC_dz_aff + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc00 = FireFlyOffsetFreeMPC_dz_cc + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd00 = FireFlyOffsetFreeMPC_rd + 0;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd00[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost00 = FireFlyOffsetFreeMPC_grad_cost + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq00 = FireFlyOffsetFreeMPC_grad_eq + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq00 = FireFlyOffsetFreeMPC_grad_ineq + 0;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv00[17];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_C00[476] = {1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9990000499983334E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9990000499983334E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9990000499983334E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -4.8249854442064527E-004, 0.0000000000000000E+000, 0.0000000000000000E+000, -9.5709381110608033E-002, 0.0000000000000000E+000, 9.5175750335964016E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 4.8192863240059008E-004, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.5539861361644718E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.4836009671115240E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662514E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662514E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662514E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -6.5637571222153614E-006, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.9610569514738377E-003, 0.0000000000000000E+000, 3.9655332238375807E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 6.5086577006017774E-006, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.9440218764483696E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 3.9287638422155269E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v00 = FireFlyOffsetFreeMPC_v + 0;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re00[28];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta00[28];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc00[28];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff00 = FireFlyOffsetFreeMPC_dv_aff + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc00 = FireFlyOffsetFreeMPC_dv_cc + 0;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V00[476];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd00[406];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld00[406];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy00[28];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy00[28];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb00[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx00[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb00 = FireFlyOffsetFreeMPC_l + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb00 = FireFlyOffsetFreeMPC_s + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb00 = FireFlyOffsetFreeMPC_lbys + 0;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb00[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff00 = FireFlyOffsetFreeMPC_dl_aff + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff00 = FireFlyOffsetFreeMPC_ds_aff + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc00 = FireFlyOffsetFreeMPC_dl_cc + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc00 = FireFlyOffsetFreeMPC_ds_cc + 0;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl00 = FireFlyOffsetFreeMPC_ccrhs + 0;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub00[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx00[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub00 = FireFlyOffsetFreeMPC_l + 6;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub00 = FireFlyOffsetFreeMPC_s + 6;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub00 = FireFlyOffsetFreeMPC_lbys + 6;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub00[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff00 = FireFlyOffsetFreeMPC_dl_aff + 6;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff00 = FireFlyOffsetFreeMPC_ds_aff + 6;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc00 = FireFlyOffsetFreeMPC_dl_cc + 6;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc00 = FireFlyOffsetFreeMPC_ds_cc + 6;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub00 = FireFlyOffsetFreeMPC_ccrhs + 6;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi00[153];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z01 = FireFlyOffsetFreeMPC_z + 17;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff01 = FireFlyOffsetFreeMPC_dz_aff + 17;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc01 = FireFlyOffsetFreeMPC_dz_cc + 17;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd01 = FireFlyOffsetFreeMPC_rd + 17;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd01[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost01 = FireFlyOffsetFreeMPC_grad_cost + 17;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq01 = FireFlyOffsetFreeMPC_grad_eq + 17;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq01 = FireFlyOffsetFreeMPC_grad_ineq + 17;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv01[17];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_C01[238] = {1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9990000499983334E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9990000499983334E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9990000499983334E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -4.8249854442064527E-004, 0.0000000000000000E+000, 0.0000000000000000E+000, -9.5709381110608033E-002, 0.0000000000000000E+000, 9.5175750335964016E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
4.8192863240059008E-004, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.5539861361644718E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.4836009671115240E-001, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662514E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662514E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662514E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -6.5637571222153614E-006, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.9610569514738377E-003, 0.0000000000000000E+000, 3.9655332238375807E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
6.5086577006017774E-006, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.9440218764483696E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 3.9287638422155269E-002, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 4.9998333374999167E-005, 0.0000000000000000E+000, 0.0000000000000000E+000, 9.9995000166662497E-003, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v01 = FireFlyOffsetFreeMPC_v + 28;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re01[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta01[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc01[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff01 = FireFlyOffsetFreeMPC_dv_aff + 28;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc01 = FireFlyOffsetFreeMPC_dv_cc + 28;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V01[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd01[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld01[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy01[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy01[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c01[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb01[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx01[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb01 = FireFlyOffsetFreeMPC_l + 12;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb01 = FireFlyOffsetFreeMPC_s + 12;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb01 = FireFlyOffsetFreeMPC_lbys + 12;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb01[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff01 = FireFlyOffsetFreeMPC_dl_aff + 12;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff01 = FireFlyOffsetFreeMPC_ds_aff + 12;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc01 = FireFlyOffsetFreeMPC_dl_cc + 12;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc01 = FireFlyOffsetFreeMPC_ds_cc + 12;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl01 = FireFlyOffsetFreeMPC_ccrhs + 12;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub01[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx01[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub01 = FireFlyOffsetFreeMPC_l + 18;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub01 = FireFlyOffsetFreeMPC_s + 18;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub01 = FireFlyOffsetFreeMPC_lbys + 18;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub01[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff01 = FireFlyOffsetFreeMPC_dl_aff + 18;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff01 = FireFlyOffsetFreeMPC_ds_aff + 18;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc01 = FireFlyOffsetFreeMPC_dl_cc + 18;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc01 = FireFlyOffsetFreeMPC_ds_cc + 18;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub01 = FireFlyOffsetFreeMPC_ccrhs + 18;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi01[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_D01[476] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W01[476];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd01[392];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd01[392];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z02 = FireFlyOffsetFreeMPC_z + 34;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff02 = FireFlyOffsetFreeMPC_dz_aff + 34;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc02 = FireFlyOffsetFreeMPC_dz_cc + 34;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd02 = FireFlyOffsetFreeMPC_rd + 34;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd02[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost02 = FireFlyOffsetFreeMPC_grad_cost + 34;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq02 = FireFlyOffsetFreeMPC_grad_eq + 34;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq02 = FireFlyOffsetFreeMPC_grad_ineq + 34;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv02[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v02 = FireFlyOffsetFreeMPC_v + 42;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re02[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta02[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc02[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff02 = FireFlyOffsetFreeMPC_dv_aff + 42;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc02 = FireFlyOffsetFreeMPC_dv_cc + 42;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V02[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd02[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld02[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy02[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy02[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c02[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb02[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx02[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb02 = FireFlyOffsetFreeMPC_l + 24;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb02 = FireFlyOffsetFreeMPC_s + 24;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb02 = FireFlyOffsetFreeMPC_lbys + 24;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb02[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff02 = FireFlyOffsetFreeMPC_dl_aff + 24;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff02 = FireFlyOffsetFreeMPC_ds_aff + 24;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc02 = FireFlyOffsetFreeMPC_dl_cc + 24;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc02 = FireFlyOffsetFreeMPC_ds_cc + 24;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl02 = FireFlyOffsetFreeMPC_ccrhs + 24;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub02[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx02[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub02 = FireFlyOffsetFreeMPC_l + 30;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub02 = FireFlyOffsetFreeMPC_s + 30;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub02 = FireFlyOffsetFreeMPC_lbys + 30;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub02[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff02 = FireFlyOffsetFreeMPC_dl_aff + 30;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff02 = FireFlyOffsetFreeMPC_ds_aff + 30;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc02 = FireFlyOffsetFreeMPC_dl_cc + 30;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc02 = FireFlyOffsetFreeMPC_ds_cc + 30;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub02 = FireFlyOffsetFreeMPC_ccrhs + 30;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi02[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_D02[17] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W02[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd02[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd02[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z03 = FireFlyOffsetFreeMPC_z + 51;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff03 = FireFlyOffsetFreeMPC_dz_aff + 51;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc03 = FireFlyOffsetFreeMPC_dz_cc + 51;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd03 = FireFlyOffsetFreeMPC_rd + 51;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd03[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost03 = FireFlyOffsetFreeMPC_grad_cost + 51;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq03 = FireFlyOffsetFreeMPC_grad_eq + 51;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq03 = FireFlyOffsetFreeMPC_grad_ineq + 51;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv03[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v03 = FireFlyOffsetFreeMPC_v + 56;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re03[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta03[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc03[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff03 = FireFlyOffsetFreeMPC_dv_aff + 56;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc03 = FireFlyOffsetFreeMPC_dv_cc + 56;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V03[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd03[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld03[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy03[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy03[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c03[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb03[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx03[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb03 = FireFlyOffsetFreeMPC_l + 36;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb03 = FireFlyOffsetFreeMPC_s + 36;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb03 = FireFlyOffsetFreeMPC_lbys + 36;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb03[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff03 = FireFlyOffsetFreeMPC_dl_aff + 36;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff03 = FireFlyOffsetFreeMPC_ds_aff + 36;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc03 = FireFlyOffsetFreeMPC_dl_cc + 36;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc03 = FireFlyOffsetFreeMPC_ds_cc + 36;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl03 = FireFlyOffsetFreeMPC_ccrhs + 36;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub03[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx03[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub03 = FireFlyOffsetFreeMPC_l + 42;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub03 = FireFlyOffsetFreeMPC_s + 42;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub03 = FireFlyOffsetFreeMPC_lbys + 42;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub03[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff03 = FireFlyOffsetFreeMPC_dl_aff + 42;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff03 = FireFlyOffsetFreeMPC_ds_aff + 42;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc03 = FireFlyOffsetFreeMPC_dl_cc + 42;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc03 = FireFlyOffsetFreeMPC_ds_cc + 42;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub03 = FireFlyOffsetFreeMPC_ccrhs + 42;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi03[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W03[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd03[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd03[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z04 = FireFlyOffsetFreeMPC_z + 68;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff04 = FireFlyOffsetFreeMPC_dz_aff + 68;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc04 = FireFlyOffsetFreeMPC_dz_cc + 68;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd04 = FireFlyOffsetFreeMPC_rd + 68;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd04[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost04 = FireFlyOffsetFreeMPC_grad_cost + 68;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq04 = FireFlyOffsetFreeMPC_grad_eq + 68;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq04 = FireFlyOffsetFreeMPC_grad_ineq + 68;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv04[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v04 = FireFlyOffsetFreeMPC_v + 70;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re04[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta04[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc04[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff04 = FireFlyOffsetFreeMPC_dv_aff + 70;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc04 = FireFlyOffsetFreeMPC_dv_cc + 70;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V04[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd04[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld04[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy04[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy04[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c04[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb04[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx04[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb04 = FireFlyOffsetFreeMPC_l + 48;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb04 = FireFlyOffsetFreeMPC_s + 48;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb04 = FireFlyOffsetFreeMPC_lbys + 48;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb04[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff04 = FireFlyOffsetFreeMPC_dl_aff + 48;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff04 = FireFlyOffsetFreeMPC_ds_aff + 48;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc04 = FireFlyOffsetFreeMPC_dl_cc + 48;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc04 = FireFlyOffsetFreeMPC_ds_cc + 48;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl04 = FireFlyOffsetFreeMPC_ccrhs + 48;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub04[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx04[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub04 = FireFlyOffsetFreeMPC_l + 54;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub04 = FireFlyOffsetFreeMPC_s + 54;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub04 = FireFlyOffsetFreeMPC_lbys + 54;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub04[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff04 = FireFlyOffsetFreeMPC_dl_aff + 54;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff04 = FireFlyOffsetFreeMPC_ds_aff + 54;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc04 = FireFlyOffsetFreeMPC_dl_cc + 54;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc04 = FireFlyOffsetFreeMPC_ds_cc + 54;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub04 = FireFlyOffsetFreeMPC_ccrhs + 54;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi04[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W04[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd04[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd04[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z05 = FireFlyOffsetFreeMPC_z + 85;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff05 = FireFlyOffsetFreeMPC_dz_aff + 85;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc05 = FireFlyOffsetFreeMPC_dz_cc + 85;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd05 = FireFlyOffsetFreeMPC_rd + 85;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd05[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost05 = FireFlyOffsetFreeMPC_grad_cost + 85;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq05 = FireFlyOffsetFreeMPC_grad_eq + 85;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq05 = FireFlyOffsetFreeMPC_grad_ineq + 85;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv05[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v05 = FireFlyOffsetFreeMPC_v + 84;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re05[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta05[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc05[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff05 = FireFlyOffsetFreeMPC_dv_aff + 84;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc05 = FireFlyOffsetFreeMPC_dv_cc + 84;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V05[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd05[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld05[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy05[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy05[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c05[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb05[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx05[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb05 = FireFlyOffsetFreeMPC_l + 60;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb05 = FireFlyOffsetFreeMPC_s + 60;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb05 = FireFlyOffsetFreeMPC_lbys + 60;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb05[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff05 = FireFlyOffsetFreeMPC_dl_aff + 60;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff05 = FireFlyOffsetFreeMPC_ds_aff + 60;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc05 = FireFlyOffsetFreeMPC_dl_cc + 60;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc05 = FireFlyOffsetFreeMPC_ds_cc + 60;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl05 = FireFlyOffsetFreeMPC_ccrhs + 60;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub05[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx05[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub05 = FireFlyOffsetFreeMPC_l + 66;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub05 = FireFlyOffsetFreeMPC_s + 66;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub05 = FireFlyOffsetFreeMPC_lbys + 66;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub05[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff05 = FireFlyOffsetFreeMPC_dl_aff + 66;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff05 = FireFlyOffsetFreeMPC_ds_aff + 66;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc05 = FireFlyOffsetFreeMPC_dl_cc + 66;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc05 = FireFlyOffsetFreeMPC_ds_cc + 66;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub05 = FireFlyOffsetFreeMPC_ccrhs + 66;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi05[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W05[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd05[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd05[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z06 = FireFlyOffsetFreeMPC_z + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff06 = FireFlyOffsetFreeMPC_dz_aff + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc06 = FireFlyOffsetFreeMPC_dz_cc + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd06 = FireFlyOffsetFreeMPC_rd + 102;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd06[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost06 = FireFlyOffsetFreeMPC_grad_cost + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq06 = FireFlyOffsetFreeMPC_grad_eq + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq06 = FireFlyOffsetFreeMPC_grad_ineq + 102;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv06[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v06 = FireFlyOffsetFreeMPC_v + 98;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re06[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta06[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc06[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff06 = FireFlyOffsetFreeMPC_dv_aff + 98;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc06 = FireFlyOffsetFreeMPC_dv_cc + 98;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V06[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd06[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld06[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy06[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy06[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c06[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb06[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx06[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb06 = FireFlyOffsetFreeMPC_l + 72;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb06 = FireFlyOffsetFreeMPC_s + 72;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb06 = FireFlyOffsetFreeMPC_lbys + 72;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb06[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff06 = FireFlyOffsetFreeMPC_dl_aff + 72;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff06 = FireFlyOffsetFreeMPC_ds_aff + 72;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc06 = FireFlyOffsetFreeMPC_dl_cc + 72;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc06 = FireFlyOffsetFreeMPC_ds_cc + 72;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl06 = FireFlyOffsetFreeMPC_ccrhs + 72;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub06[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx06[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub06 = FireFlyOffsetFreeMPC_l + 78;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub06 = FireFlyOffsetFreeMPC_s + 78;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub06 = FireFlyOffsetFreeMPC_lbys + 78;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub06[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff06 = FireFlyOffsetFreeMPC_dl_aff + 78;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff06 = FireFlyOffsetFreeMPC_ds_aff + 78;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc06 = FireFlyOffsetFreeMPC_dl_cc + 78;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc06 = FireFlyOffsetFreeMPC_ds_cc + 78;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub06 = FireFlyOffsetFreeMPC_ccrhs + 78;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi06[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W06[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd06[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd06[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z07 = FireFlyOffsetFreeMPC_z + 119;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff07 = FireFlyOffsetFreeMPC_dz_aff + 119;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc07 = FireFlyOffsetFreeMPC_dz_cc + 119;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd07 = FireFlyOffsetFreeMPC_rd + 119;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd07[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost07 = FireFlyOffsetFreeMPC_grad_cost + 119;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq07 = FireFlyOffsetFreeMPC_grad_eq + 119;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq07 = FireFlyOffsetFreeMPC_grad_ineq + 119;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv07[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v07 = FireFlyOffsetFreeMPC_v + 112;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re07[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta07[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc07[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff07 = FireFlyOffsetFreeMPC_dv_aff + 112;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc07 = FireFlyOffsetFreeMPC_dv_cc + 112;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V07[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd07[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld07[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy07[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy07[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c07[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb07[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx07[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb07 = FireFlyOffsetFreeMPC_l + 84;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb07 = FireFlyOffsetFreeMPC_s + 84;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb07 = FireFlyOffsetFreeMPC_lbys + 84;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb07[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff07 = FireFlyOffsetFreeMPC_dl_aff + 84;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff07 = FireFlyOffsetFreeMPC_ds_aff + 84;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc07 = FireFlyOffsetFreeMPC_dl_cc + 84;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc07 = FireFlyOffsetFreeMPC_ds_cc + 84;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl07 = FireFlyOffsetFreeMPC_ccrhs + 84;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub07[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx07[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub07 = FireFlyOffsetFreeMPC_l + 90;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub07 = FireFlyOffsetFreeMPC_s + 90;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub07 = FireFlyOffsetFreeMPC_lbys + 90;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub07[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff07 = FireFlyOffsetFreeMPC_dl_aff + 90;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff07 = FireFlyOffsetFreeMPC_ds_aff + 90;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc07 = FireFlyOffsetFreeMPC_dl_cc + 90;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc07 = FireFlyOffsetFreeMPC_ds_cc + 90;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub07 = FireFlyOffsetFreeMPC_ccrhs + 90;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi07[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W07[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd07[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd07[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z08 = FireFlyOffsetFreeMPC_z + 136;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff08 = FireFlyOffsetFreeMPC_dz_aff + 136;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc08 = FireFlyOffsetFreeMPC_dz_cc + 136;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd08 = FireFlyOffsetFreeMPC_rd + 136;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd08[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost08 = FireFlyOffsetFreeMPC_grad_cost + 136;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq08 = FireFlyOffsetFreeMPC_grad_eq + 136;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq08 = FireFlyOffsetFreeMPC_grad_ineq + 136;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv08[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v08 = FireFlyOffsetFreeMPC_v + 126;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re08[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta08[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc08[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff08 = FireFlyOffsetFreeMPC_dv_aff + 126;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc08 = FireFlyOffsetFreeMPC_dv_cc + 126;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V08[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd08[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld08[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy08[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy08[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c08[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb08[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx08[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb08 = FireFlyOffsetFreeMPC_l + 96;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb08 = FireFlyOffsetFreeMPC_s + 96;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb08 = FireFlyOffsetFreeMPC_lbys + 96;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb08[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff08 = FireFlyOffsetFreeMPC_dl_aff + 96;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff08 = FireFlyOffsetFreeMPC_ds_aff + 96;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc08 = FireFlyOffsetFreeMPC_dl_cc + 96;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc08 = FireFlyOffsetFreeMPC_ds_cc + 96;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl08 = FireFlyOffsetFreeMPC_ccrhs + 96;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub08[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx08[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub08 = FireFlyOffsetFreeMPC_l + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub08 = FireFlyOffsetFreeMPC_s + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub08 = FireFlyOffsetFreeMPC_lbys + 102;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub08[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff08 = FireFlyOffsetFreeMPC_dl_aff + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff08 = FireFlyOffsetFreeMPC_ds_aff + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc08 = FireFlyOffsetFreeMPC_dl_cc + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc08 = FireFlyOffsetFreeMPC_ds_cc + 102;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub08 = FireFlyOffsetFreeMPC_ccrhs + 102;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi08[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W08[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd08[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd08[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z09 = FireFlyOffsetFreeMPC_z + 153;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff09 = FireFlyOffsetFreeMPC_dz_aff + 153;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc09 = FireFlyOffsetFreeMPC_dz_cc + 153;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd09 = FireFlyOffsetFreeMPC_rd + 153;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd09[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost09 = FireFlyOffsetFreeMPC_grad_cost + 153;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq09 = FireFlyOffsetFreeMPC_grad_eq + 153;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq09 = FireFlyOffsetFreeMPC_grad_ineq + 153;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv09[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v09 = FireFlyOffsetFreeMPC_v + 140;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re09[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta09[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc09[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff09 = FireFlyOffsetFreeMPC_dv_aff + 140;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc09 = FireFlyOffsetFreeMPC_dv_cc + 140;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V09[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd09[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld09[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy09[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy09[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c09[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb09[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx09[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb09 = FireFlyOffsetFreeMPC_l + 108;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb09 = FireFlyOffsetFreeMPC_s + 108;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb09 = FireFlyOffsetFreeMPC_lbys + 108;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb09[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff09 = FireFlyOffsetFreeMPC_dl_aff + 108;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff09 = FireFlyOffsetFreeMPC_ds_aff + 108;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc09 = FireFlyOffsetFreeMPC_dl_cc + 108;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc09 = FireFlyOffsetFreeMPC_ds_cc + 108;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl09 = FireFlyOffsetFreeMPC_ccrhs + 108;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub09[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx09[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub09 = FireFlyOffsetFreeMPC_l + 114;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub09 = FireFlyOffsetFreeMPC_s + 114;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub09 = FireFlyOffsetFreeMPC_lbys + 114;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub09[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff09 = FireFlyOffsetFreeMPC_dl_aff + 114;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff09 = FireFlyOffsetFreeMPC_ds_aff + 114;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc09 = FireFlyOffsetFreeMPC_dl_cc + 114;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc09 = FireFlyOffsetFreeMPC_ds_cc + 114;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub09 = FireFlyOffsetFreeMPC_ccrhs + 114;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi09[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W09[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd09[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd09[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z10 = FireFlyOffsetFreeMPC_z + 170;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff10 = FireFlyOffsetFreeMPC_dz_aff + 170;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc10 = FireFlyOffsetFreeMPC_dz_cc + 170;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd10 = FireFlyOffsetFreeMPC_rd + 170;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd10[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost10 = FireFlyOffsetFreeMPC_grad_cost + 170;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq10 = FireFlyOffsetFreeMPC_grad_eq + 170;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq10 = FireFlyOffsetFreeMPC_grad_ineq + 170;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv10[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v10 = FireFlyOffsetFreeMPC_v + 154;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re10[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta10[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc10[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff10 = FireFlyOffsetFreeMPC_dv_aff + 154;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc10 = FireFlyOffsetFreeMPC_dv_cc + 154;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V10[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd10[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld10[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy10[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy10[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c10[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb10[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx10[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb10 = FireFlyOffsetFreeMPC_l + 120;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb10 = FireFlyOffsetFreeMPC_s + 120;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb10 = FireFlyOffsetFreeMPC_lbys + 120;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb10[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff10 = FireFlyOffsetFreeMPC_dl_aff + 120;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff10 = FireFlyOffsetFreeMPC_ds_aff + 120;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc10 = FireFlyOffsetFreeMPC_dl_cc + 120;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc10 = FireFlyOffsetFreeMPC_ds_cc + 120;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl10 = FireFlyOffsetFreeMPC_ccrhs + 120;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub10[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx10[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub10 = FireFlyOffsetFreeMPC_l + 126;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub10 = FireFlyOffsetFreeMPC_s + 126;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub10 = FireFlyOffsetFreeMPC_lbys + 126;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub10[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff10 = FireFlyOffsetFreeMPC_dl_aff + 126;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff10 = FireFlyOffsetFreeMPC_ds_aff + 126;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc10 = FireFlyOffsetFreeMPC_dl_cc + 126;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc10 = FireFlyOffsetFreeMPC_ds_cc + 126;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub10 = FireFlyOffsetFreeMPC_ccrhs + 126;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi10[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W10[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd10[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd10[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z11 = FireFlyOffsetFreeMPC_z + 187;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff11 = FireFlyOffsetFreeMPC_dz_aff + 187;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc11 = FireFlyOffsetFreeMPC_dz_cc + 187;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd11 = FireFlyOffsetFreeMPC_rd + 187;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd11[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost11 = FireFlyOffsetFreeMPC_grad_cost + 187;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq11 = FireFlyOffsetFreeMPC_grad_eq + 187;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq11 = FireFlyOffsetFreeMPC_grad_ineq + 187;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv11[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v11 = FireFlyOffsetFreeMPC_v + 168;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re11[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta11[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc11[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff11 = FireFlyOffsetFreeMPC_dv_aff + 168;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc11 = FireFlyOffsetFreeMPC_dv_cc + 168;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V11[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd11[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld11[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy11[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy11[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c11[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb11[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx11[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb11 = FireFlyOffsetFreeMPC_l + 132;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb11 = FireFlyOffsetFreeMPC_s + 132;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb11 = FireFlyOffsetFreeMPC_lbys + 132;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb11[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff11 = FireFlyOffsetFreeMPC_dl_aff + 132;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff11 = FireFlyOffsetFreeMPC_ds_aff + 132;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc11 = FireFlyOffsetFreeMPC_dl_cc + 132;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc11 = FireFlyOffsetFreeMPC_ds_cc + 132;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl11 = FireFlyOffsetFreeMPC_ccrhs + 132;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub11[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx11[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub11 = FireFlyOffsetFreeMPC_l + 138;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub11 = FireFlyOffsetFreeMPC_s + 138;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub11 = FireFlyOffsetFreeMPC_lbys + 138;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub11[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff11 = FireFlyOffsetFreeMPC_dl_aff + 138;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff11 = FireFlyOffsetFreeMPC_ds_aff + 138;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc11 = FireFlyOffsetFreeMPC_dl_cc + 138;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc11 = FireFlyOffsetFreeMPC_ds_cc + 138;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub11 = FireFlyOffsetFreeMPC_ccrhs + 138;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi11[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W11[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd11[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd11[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z12 = FireFlyOffsetFreeMPC_z + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff12 = FireFlyOffsetFreeMPC_dz_aff + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc12 = FireFlyOffsetFreeMPC_dz_cc + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd12 = FireFlyOffsetFreeMPC_rd + 204;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd12[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost12 = FireFlyOffsetFreeMPC_grad_cost + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq12 = FireFlyOffsetFreeMPC_grad_eq + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq12 = FireFlyOffsetFreeMPC_grad_ineq + 204;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv12[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v12 = FireFlyOffsetFreeMPC_v + 182;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re12[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta12[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc12[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff12 = FireFlyOffsetFreeMPC_dv_aff + 182;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc12 = FireFlyOffsetFreeMPC_dv_cc + 182;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V12[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd12[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld12[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy12[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy12[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c12[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb12[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx12[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb12 = FireFlyOffsetFreeMPC_l + 144;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb12 = FireFlyOffsetFreeMPC_s + 144;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb12 = FireFlyOffsetFreeMPC_lbys + 144;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb12[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff12 = FireFlyOffsetFreeMPC_dl_aff + 144;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff12 = FireFlyOffsetFreeMPC_ds_aff + 144;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc12 = FireFlyOffsetFreeMPC_dl_cc + 144;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc12 = FireFlyOffsetFreeMPC_ds_cc + 144;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl12 = FireFlyOffsetFreeMPC_ccrhs + 144;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub12[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx12[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub12 = FireFlyOffsetFreeMPC_l + 150;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub12 = FireFlyOffsetFreeMPC_s + 150;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub12 = FireFlyOffsetFreeMPC_lbys + 150;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub12[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff12 = FireFlyOffsetFreeMPC_dl_aff + 150;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff12 = FireFlyOffsetFreeMPC_ds_aff + 150;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc12 = FireFlyOffsetFreeMPC_dl_cc + 150;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc12 = FireFlyOffsetFreeMPC_ds_cc + 150;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub12 = FireFlyOffsetFreeMPC_ccrhs + 150;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi12[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W12[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd12[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd12[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z13 = FireFlyOffsetFreeMPC_z + 221;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff13 = FireFlyOffsetFreeMPC_dz_aff + 221;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc13 = FireFlyOffsetFreeMPC_dz_cc + 221;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd13 = FireFlyOffsetFreeMPC_rd + 221;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd13[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost13 = FireFlyOffsetFreeMPC_grad_cost + 221;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq13 = FireFlyOffsetFreeMPC_grad_eq + 221;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq13 = FireFlyOffsetFreeMPC_grad_ineq + 221;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv13[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v13 = FireFlyOffsetFreeMPC_v + 196;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re13[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta13[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc13[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff13 = FireFlyOffsetFreeMPC_dv_aff + 196;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc13 = FireFlyOffsetFreeMPC_dv_cc + 196;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V13[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd13[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld13[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy13[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy13[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c13[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb13[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx13[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb13 = FireFlyOffsetFreeMPC_l + 156;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb13 = FireFlyOffsetFreeMPC_s + 156;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb13 = FireFlyOffsetFreeMPC_lbys + 156;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb13[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff13 = FireFlyOffsetFreeMPC_dl_aff + 156;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff13 = FireFlyOffsetFreeMPC_ds_aff + 156;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc13 = FireFlyOffsetFreeMPC_dl_cc + 156;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc13 = FireFlyOffsetFreeMPC_ds_cc + 156;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl13 = FireFlyOffsetFreeMPC_ccrhs + 156;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub13[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx13[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub13 = FireFlyOffsetFreeMPC_l + 162;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub13 = FireFlyOffsetFreeMPC_s + 162;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub13 = FireFlyOffsetFreeMPC_lbys + 162;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub13[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff13 = FireFlyOffsetFreeMPC_dl_aff + 162;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff13 = FireFlyOffsetFreeMPC_ds_aff + 162;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc13 = FireFlyOffsetFreeMPC_dl_cc + 162;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc13 = FireFlyOffsetFreeMPC_ds_cc + 162;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub13 = FireFlyOffsetFreeMPC_ccrhs + 162;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi13[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W13[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd13[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd13[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z14 = FireFlyOffsetFreeMPC_z + 238;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff14 = FireFlyOffsetFreeMPC_dz_aff + 238;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc14 = FireFlyOffsetFreeMPC_dz_cc + 238;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd14 = FireFlyOffsetFreeMPC_rd + 238;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd14[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost14 = FireFlyOffsetFreeMPC_grad_cost + 238;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq14 = FireFlyOffsetFreeMPC_grad_eq + 238;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq14 = FireFlyOffsetFreeMPC_grad_ineq + 238;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv14[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v14 = FireFlyOffsetFreeMPC_v + 210;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re14[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta14[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc14[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff14 = FireFlyOffsetFreeMPC_dv_aff + 210;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc14 = FireFlyOffsetFreeMPC_dv_cc + 210;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V14[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd14[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld14[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy14[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy14[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c14[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb14[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx14[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb14 = FireFlyOffsetFreeMPC_l + 168;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb14 = FireFlyOffsetFreeMPC_s + 168;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb14 = FireFlyOffsetFreeMPC_lbys + 168;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb14[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff14 = FireFlyOffsetFreeMPC_dl_aff + 168;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff14 = FireFlyOffsetFreeMPC_ds_aff + 168;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc14 = FireFlyOffsetFreeMPC_dl_cc + 168;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc14 = FireFlyOffsetFreeMPC_ds_cc + 168;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl14 = FireFlyOffsetFreeMPC_ccrhs + 168;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub14[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx14[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub14 = FireFlyOffsetFreeMPC_l + 174;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub14 = FireFlyOffsetFreeMPC_s + 174;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub14 = FireFlyOffsetFreeMPC_lbys + 174;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub14[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff14 = FireFlyOffsetFreeMPC_dl_aff + 174;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff14 = FireFlyOffsetFreeMPC_ds_aff + 174;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc14 = FireFlyOffsetFreeMPC_dl_cc + 174;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc14 = FireFlyOffsetFreeMPC_ds_cc + 174;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub14 = FireFlyOffsetFreeMPC_ccrhs + 174;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi14[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W14[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd14[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd14[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z15 = FireFlyOffsetFreeMPC_z + 255;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff15 = FireFlyOffsetFreeMPC_dz_aff + 255;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc15 = FireFlyOffsetFreeMPC_dz_cc + 255;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd15 = FireFlyOffsetFreeMPC_rd + 255;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd15[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost15 = FireFlyOffsetFreeMPC_grad_cost + 255;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq15 = FireFlyOffsetFreeMPC_grad_eq + 255;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq15 = FireFlyOffsetFreeMPC_grad_ineq + 255;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv15[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v15 = FireFlyOffsetFreeMPC_v + 224;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re15[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta15[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc15[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff15 = FireFlyOffsetFreeMPC_dv_aff + 224;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc15 = FireFlyOffsetFreeMPC_dv_cc + 224;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V15[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd15[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld15[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy15[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy15[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c15[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb15[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx15[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb15 = FireFlyOffsetFreeMPC_l + 180;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb15 = FireFlyOffsetFreeMPC_s + 180;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb15 = FireFlyOffsetFreeMPC_lbys + 180;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb15[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff15 = FireFlyOffsetFreeMPC_dl_aff + 180;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff15 = FireFlyOffsetFreeMPC_ds_aff + 180;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc15 = FireFlyOffsetFreeMPC_dl_cc + 180;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc15 = FireFlyOffsetFreeMPC_ds_cc + 180;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl15 = FireFlyOffsetFreeMPC_ccrhs + 180;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub15[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx15[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub15 = FireFlyOffsetFreeMPC_l + 186;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub15 = FireFlyOffsetFreeMPC_s + 186;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub15 = FireFlyOffsetFreeMPC_lbys + 186;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub15[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff15 = FireFlyOffsetFreeMPC_dl_aff + 186;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff15 = FireFlyOffsetFreeMPC_ds_aff + 186;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc15 = FireFlyOffsetFreeMPC_dl_cc + 186;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc15 = FireFlyOffsetFreeMPC_ds_cc + 186;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub15 = FireFlyOffsetFreeMPC_ccrhs + 186;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi15[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W15[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd15[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd15[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z16 = FireFlyOffsetFreeMPC_z + 272;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff16 = FireFlyOffsetFreeMPC_dz_aff + 272;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc16 = FireFlyOffsetFreeMPC_dz_cc + 272;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd16 = FireFlyOffsetFreeMPC_rd + 272;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd16[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost16 = FireFlyOffsetFreeMPC_grad_cost + 272;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq16 = FireFlyOffsetFreeMPC_grad_eq + 272;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq16 = FireFlyOffsetFreeMPC_grad_ineq + 272;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv16[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v16 = FireFlyOffsetFreeMPC_v + 238;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re16[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta16[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc16[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff16 = FireFlyOffsetFreeMPC_dv_aff + 238;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc16 = FireFlyOffsetFreeMPC_dv_cc + 238;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V16[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd16[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld16[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy16[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy16[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c16[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb16[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx16[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb16 = FireFlyOffsetFreeMPC_l + 192;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb16 = FireFlyOffsetFreeMPC_s + 192;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb16 = FireFlyOffsetFreeMPC_lbys + 192;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb16[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff16 = FireFlyOffsetFreeMPC_dl_aff + 192;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff16 = FireFlyOffsetFreeMPC_ds_aff + 192;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc16 = FireFlyOffsetFreeMPC_dl_cc + 192;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc16 = FireFlyOffsetFreeMPC_ds_cc + 192;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl16 = FireFlyOffsetFreeMPC_ccrhs + 192;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub16[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx16[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub16 = FireFlyOffsetFreeMPC_l + 198;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub16 = FireFlyOffsetFreeMPC_s + 198;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub16 = FireFlyOffsetFreeMPC_lbys + 198;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub16[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff16 = FireFlyOffsetFreeMPC_dl_aff + 198;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff16 = FireFlyOffsetFreeMPC_ds_aff + 198;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc16 = FireFlyOffsetFreeMPC_dl_cc + 198;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc16 = FireFlyOffsetFreeMPC_ds_cc + 198;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub16 = FireFlyOffsetFreeMPC_ccrhs + 198;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi16[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W16[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd16[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd16[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z17 = FireFlyOffsetFreeMPC_z + 289;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff17 = FireFlyOffsetFreeMPC_dz_aff + 289;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc17 = FireFlyOffsetFreeMPC_dz_cc + 289;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd17 = FireFlyOffsetFreeMPC_rd + 289;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd17[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost17 = FireFlyOffsetFreeMPC_grad_cost + 289;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq17 = FireFlyOffsetFreeMPC_grad_eq + 289;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq17 = FireFlyOffsetFreeMPC_grad_ineq + 289;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv17[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v17 = FireFlyOffsetFreeMPC_v + 252;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re17[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta17[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc17[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff17 = FireFlyOffsetFreeMPC_dv_aff + 252;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc17 = FireFlyOffsetFreeMPC_dv_cc + 252;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V17[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd17[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld17[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy17[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy17[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c17[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb17[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx17[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb17 = FireFlyOffsetFreeMPC_l + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb17 = FireFlyOffsetFreeMPC_s + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb17 = FireFlyOffsetFreeMPC_lbys + 204;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb17[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff17 = FireFlyOffsetFreeMPC_dl_aff + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff17 = FireFlyOffsetFreeMPC_ds_aff + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc17 = FireFlyOffsetFreeMPC_dl_cc + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc17 = FireFlyOffsetFreeMPC_ds_cc + 204;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl17 = FireFlyOffsetFreeMPC_ccrhs + 204;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub17[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx17[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub17 = FireFlyOffsetFreeMPC_l + 210;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub17 = FireFlyOffsetFreeMPC_s + 210;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub17 = FireFlyOffsetFreeMPC_lbys + 210;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub17[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff17 = FireFlyOffsetFreeMPC_dl_aff + 210;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff17 = FireFlyOffsetFreeMPC_ds_aff + 210;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc17 = FireFlyOffsetFreeMPC_dl_cc + 210;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc17 = FireFlyOffsetFreeMPC_ds_cc + 210;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub17 = FireFlyOffsetFreeMPC_ccrhs + 210;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi17[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W17[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd17[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd17[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z18 = FireFlyOffsetFreeMPC_z + 306;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff18 = FireFlyOffsetFreeMPC_dz_aff + 306;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc18 = FireFlyOffsetFreeMPC_dz_cc + 306;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd18 = FireFlyOffsetFreeMPC_rd + 306;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd18[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost18 = FireFlyOffsetFreeMPC_grad_cost + 306;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq18 = FireFlyOffsetFreeMPC_grad_eq + 306;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq18 = FireFlyOffsetFreeMPC_grad_ineq + 306;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv18[17];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_v18 = FireFlyOffsetFreeMPC_v + 266;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_re18[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_beta18[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_betacc18[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvaff18 = FireFlyOffsetFreeMPC_dv_aff + 266;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dvcc18 = FireFlyOffsetFreeMPC_dv_cc + 266;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_V18[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Yd18[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ld18[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_yy18[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_bmy18[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_c18[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb18[6] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000, -8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx18[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb18 = FireFlyOffsetFreeMPC_l + 216;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb18 = FireFlyOffsetFreeMPC_s + 216;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb18 = FireFlyOffsetFreeMPC_lbys + 216;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb18[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff18 = FireFlyOffsetFreeMPC_dl_aff + 216;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff18 = FireFlyOffsetFreeMPC_ds_aff + 216;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc18 = FireFlyOffsetFreeMPC_dl_cc + 216;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc18 = FireFlyOffsetFreeMPC_ds_cc + 216;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl18 = FireFlyOffsetFreeMPC_ccrhs + 216;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub18[6] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001, 8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx18[6] = {11, 12, 13, 14, 15, 16};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub18 = FireFlyOffsetFreeMPC_l + 222;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub18 = FireFlyOffsetFreeMPC_s + 222;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub18 = FireFlyOffsetFreeMPC_lbys + 222;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub18[6];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff18 = FireFlyOffsetFreeMPC_dl_aff + 222;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff18 = FireFlyOffsetFreeMPC_ds_aff + 222;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc18 = FireFlyOffsetFreeMPC_dl_cc + 222;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc18 = FireFlyOffsetFreeMPC_ds_cc + 222;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub18 = FireFlyOffsetFreeMPC_ccrhs + 222;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi18[153];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W18[238];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Ysd18[196];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lsd18[196];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_z19 = FireFlyOffsetFreeMPC_z + 323;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzaff19 = FireFlyOffsetFreeMPC_dz_aff + 323;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dzcc19 = FireFlyOffsetFreeMPC_dz_cc + 323;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_rd19 = FireFlyOffsetFreeMPC_rd + 323;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Lbyrd19[14];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_cost19 = FireFlyOffsetFreeMPC_grad_cost + 323;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_eq19 = FireFlyOffsetFreeMPC_grad_eq + 323;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_grad_ineq19 = FireFlyOffsetFreeMPC_grad_ineq + 323;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ctv19[14];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_lb19[3] = {-8.7266462599716477E-001, -8.7266462599716477E-001, -9.8100000000000005E+000};
int FireFlyOffsetFreeMPC_lbIdx19[3] = {11, 12, 13};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llb19 = FireFlyOffsetFreeMPC_l + 228;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_slb19 = FireFlyOffsetFreeMPC_s + 228;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_llbbyslb19 = FireFlyOffsetFreeMPC_lbys + 228;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_rilb19[3];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbaff19 = FireFlyOffsetFreeMPC_dl_aff + 228;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbaff19 = FireFlyOffsetFreeMPC_ds_aff + 228;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dllbcc19 = FireFlyOffsetFreeMPC_dl_cc + 228;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dslbcc19 = FireFlyOffsetFreeMPC_ds_cc + 228;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsl19 = FireFlyOffsetFreeMPC_ccrhs + 228;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_ub19[3] = {8.7266462599716477E-001, 8.7266462599716477E-001, 3.0000000000000000E+001};
int FireFlyOffsetFreeMPC_ubIdx19[3] = {11, 12, 13};
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lub19 = FireFlyOffsetFreeMPC_l + 231;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_sub19 = FireFlyOffsetFreeMPC_s + 231;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_lubbysub19 = FireFlyOffsetFreeMPC_lbys + 231;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_riub19[3];
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubaff19 = FireFlyOffsetFreeMPC_dl_aff + 231;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubaff19 = FireFlyOffsetFreeMPC_ds_aff + 231;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dlubcc19 = FireFlyOffsetFreeMPC_dl_cc + 231;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_dsubcc19 = FireFlyOffsetFreeMPC_ds_cc + 231;
FireFlyOffsetFreeMPC_FLOAT* FireFlyOffsetFreeMPC_ccrhsub19 = FireFlyOffsetFreeMPC_ccrhs + 231;
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_Phi19[105];
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_D19[14] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
FireFlyOffsetFreeMPC_FLOAT FireFlyOffsetFreeMPC_W19[196];
FireFlyOffsetFreeMPC_FLOAT musigma;
FireFlyOffsetFreeMPC_FLOAT sigma_3rdroot;




/* SOLVER CODE --------------------------------------------------------- */
int FireFlyOffsetFreeMPC_solve(FireFlyOffsetFreeMPC_params* params, FireFlyOffsetFreeMPC_output* output, FireFlyOffsetFreeMPC_info* info)
{	
int exitcode;

#if FireFlyOffsetFreeMPC_SET_TIMING == 1
	FireFlyOffsetFreeMPC_timer solvertimer;
	FireFlyOffsetFreeMPC_tic(&solvertimer);
#endif
/* FUNCTION CALLS INTO LA LIBRARY -------------------------------------- */
info->it = 0;
FireFlyOffsetFreeMPC_LA_INITIALIZEVECTOR_337(FireFlyOffsetFreeMPC_z, 0);
FireFlyOffsetFreeMPC_LA_INITIALIZEVECTOR_280(FireFlyOffsetFreeMPC_v, 1);
FireFlyOffsetFreeMPC_LA_INITIALIZEVECTOR_234(FireFlyOffsetFreeMPC_l, 10);
FireFlyOffsetFreeMPC_LA_INITIALIZEVECTOR_234(FireFlyOffsetFreeMPC_s, 10);
info->mu = 0;
FireFlyOffsetFreeMPC_LA_DOTACC_234(FireFlyOffsetFreeMPC_l, FireFlyOffsetFreeMPC_s, &info->mu);
info->mu /= 234;
while( 1 ){
info->pobj = 0;
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_1, FireFlyOffsetFreeMPC_z00, FireFlyOffsetFreeMPC_grad_cost00, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_2, FireFlyOffsetFreeMPC_z01, FireFlyOffsetFreeMPC_grad_cost01, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_3, FireFlyOffsetFreeMPC_z02, FireFlyOffsetFreeMPC_grad_cost02, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_4, FireFlyOffsetFreeMPC_z03, FireFlyOffsetFreeMPC_grad_cost03, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_5, FireFlyOffsetFreeMPC_z04, FireFlyOffsetFreeMPC_grad_cost04, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_6, FireFlyOffsetFreeMPC_z05, FireFlyOffsetFreeMPC_grad_cost05, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_7, FireFlyOffsetFreeMPC_z06, FireFlyOffsetFreeMPC_grad_cost06, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_8, FireFlyOffsetFreeMPC_z07, FireFlyOffsetFreeMPC_grad_cost07, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_9, FireFlyOffsetFreeMPC_z08, FireFlyOffsetFreeMPC_grad_cost08, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_10, FireFlyOffsetFreeMPC_z09, FireFlyOffsetFreeMPC_grad_cost09, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_11, FireFlyOffsetFreeMPC_z10, FireFlyOffsetFreeMPC_grad_cost10, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_12, FireFlyOffsetFreeMPC_z11, FireFlyOffsetFreeMPC_grad_cost11, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_13, FireFlyOffsetFreeMPC_z12, FireFlyOffsetFreeMPC_grad_cost12, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_14, FireFlyOffsetFreeMPC_z13, FireFlyOffsetFreeMPC_grad_cost13, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_15, FireFlyOffsetFreeMPC_z14, FireFlyOffsetFreeMPC_grad_cost14, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_16, FireFlyOffsetFreeMPC_z15, FireFlyOffsetFreeMPC_grad_cost15, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_17, FireFlyOffsetFreeMPC_z16, FireFlyOffsetFreeMPC_grad_cost16, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_18, FireFlyOffsetFreeMPC_z17, FireFlyOffsetFreeMPC_grad_cost17, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_17(params->H, params->f_19, FireFlyOffsetFreeMPC_z18, FireFlyOffsetFreeMPC_grad_cost18, &info->pobj);
FireFlyOffsetFreeMPC_LA_DENSE_QUADFCN_14(params->H_N, params->f_N, FireFlyOffsetFreeMPC_z19, FireFlyOffsetFreeMPC_grad_cost19, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB3_28_17_17(FireFlyOffsetFreeMPC_C00, FireFlyOffsetFreeMPC_z00, FireFlyOffsetFreeMPC_D01, FireFlyOffsetFreeMPC_z01, params->z1, FireFlyOffsetFreeMPC_v00, FireFlyOffsetFreeMPC_re00, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z01, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z02, FireFlyOffsetFreeMPC_c01, FireFlyOffsetFreeMPC_v01, FireFlyOffsetFreeMPC_re01, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z02, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z03, FireFlyOffsetFreeMPC_c02, FireFlyOffsetFreeMPC_v02, FireFlyOffsetFreeMPC_re02, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z03, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z04, FireFlyOffsetFreeMPC_c03, FireFlyOffsetFreeMPC_v03, FireFlyOffsetFreeMPC_re03, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z04, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z05, FireFlyOffsetFreeMPC_c04, FireFlyOffsetFreeMPC_v04, FireFlyOffsetFreeMPC_re04, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z05, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z06, FireFlyOffsetFreeMPC_c05, FireFlyOffsetFreeMPC_v05, FireFlyOffsetFreeMPC_re05, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z06, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z07, FireFlyOffsetFreeMPC_c06, FireFlyOffsetFreeMPC_v06, FireFlyOffsetFreeMPC_re06, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z07, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z08, FireFlyOffsetFreeMPC_c07, FireFlyOffsetFreeMPC_v07, FireFlyOffsetFreeMPC_re07, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z08, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z09, FireFlyOffsetFreeMPC_c08, FireFlyOffsetFreeMPC_v08, FireFlyOffsetFreeMPC_re08, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z09, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z10, FireFlyOffsetFreeMPC_c09, FireFlyOffsetFreeMPC_v09, FireFlyOffsetFreeMPC_re09, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z10, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z11, FireFlyOffsetFreeMPC_c10, FireFlyOffsetFreeMPC_v10, FireFlyOffsetFreeMPC_re10, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z11, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z12, FireFlyOffsetFreeMPC_c11, FireFlyOffsetFreeMPC_v11, FireFlyOffsetFreeMPC_re11, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z12, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z13, FireFlyOffsetFreeMPC_c12, FireFlyOffsetFreeMPC_v12, FireFlyOffsetFreeMPC_re12, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z13, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z14, FireFlyOffsetFreeMPC_c13, FireFlyOffsetFreeMPC_v13, FireFlyOffsetFreeMPC_re13, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z14, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z15, FireFlyOffsetFreeMPC_c14, FireFlyOffsetFreeMPC_v14, FireFlyOffsetFreeMPC_re14, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z15, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z16, FireFlyOffsetFreeMPC_c15, FireFlyOffsetFreeMPC_v15, FireFlyOffsetFreeMPC_re15, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z16, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z17, FireFlyOffsetFreeMPC_c16, FireFlyOffsetFreeMPC_v16, FireFlyOffsetFreeMPC_re16, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_17(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z17, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_z18, FireFlyOffsetFreeMPC_c17, FireFlyOffsetFreeMPC_v17, FireFlyOffsetFreeMPC_re17, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MVMSUB3_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_z18, FireFlyOffsetFreeMPC_D19, FireFlyOffsetFreeMPC_z19, FireFlyOffsetFreeMPC_c18, FireFlyOffsetFreeMPC_v18, FireFlyOffsetFreeMPC_re18, &info->dgap, &info->res_eq);
FireFlyOffsetFreeMPC_LA_DENSE_MTVM_28_17(FireFlyOffsetFreeMPC_C00, FireFlyOffsetFreeMPC_v00, FireFlyOffsetFreeMPC_grad_eq00);
FireFlyOffsetFreeMPC_LA_DENSE_MTVM2_14_17_28(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v01, FireFlyOffsetFreeMPC_D01, FireFlyOffsetFreeMPC_v00, FireFlyOffsetFreeMPC_grad_eq01);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v02, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v01, FireFlyOffsetFreeMPC_grad_eq02);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v03, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v02, FireFlyOffsetFreeMPC_grad_eq03);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v04, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v03, FireFlyOffsetFreeMPC_grad_eq04);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v05, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v04, FireFlyOffsetFreeMPC_grad_eq05);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v06, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v05, FireFlyOffsetFreeMPC_grad_eq06);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v07, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v06, FireFlyOffsetFreeMPC_grad_eq07);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v08, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v07, FireFlyOffsetFreeMPC_grad_eq08);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v09, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v08, FireFlyOffsetFreeMPC_grad_eq09);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v10, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v09, FireFlyOffsetFreeMPC_grad_eq10);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v11, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v10, FireFlyOffsetFreeMPC_grad_eq11);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v12, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v11, FireFlyOffsetFreeMPC_grad_eq12);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v13, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v12, FireFlyOffsetFreeMPC_grad_eq13);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v14, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v13, FireFlyOffsetFreeMPC_grad_eq14);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v15, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v14, FireFlyOffsetFreeMPC_grad_eq15);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v16, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v15, FireFlyOffsetFreeMPC_grad_eq16);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v17, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v16, FireFlyOffsetFreeMPC_grad_eq17);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_v18, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_v17, FireFlyOffsetFreeMPC_grad_eq18);
FireFlyOffsetFreeMPC_LA_DIAGZERO_MTVM_14_14(FireFlyOffsetFreeMPC_D19, FireFlyOffsetFreeMPC_v18, FireFlyOffsetFreeMPC_grad_eq19);
info->res_ineq = 0;
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb00, FireFlyOffsetFreeMPC_z00, FireFlyOffsetFreeMPC_lbIdx00, FireFlyOffsetFreeMPC_llb00, FireFlyOffsetFreeMPC_slb00, FireFlyOffsetFreeMPC_rilb00, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z00, FireFlyOffsetFreeMPC_ubIdx00, FireFlyOffsetFreeMPC_ub00, FireFlyOffsetFreeMPC_lub00, FireFlyOffsetFreeMPC_sub00, FireFlyOffsetFreeMPC_riub00, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb01, FireFlyOffsetFreeMPC_z01, FireFlyOffsetFreeMPC_lbIdx01, FireFlyOffsetFreeMPC_llb01, FireFlyOffsetFreeMPC_slb01, FireFlyOffsetFreeMPC_rilb01, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z01, FireFlyOffsetFreeMPC_ubIdx01, FireFlyOffsetFreeMPC_ub01, FireFlyOffsetFreeMPC_lub01, FireFlyOffsetFreeMPC_sub01, FireFlyOffsetFreeMPC_riub01, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb02, FireFlyOffsetFreeMPC_z02, FireFlyOffsetFreeMPC_lbIdx02, FireFlyOffsetFreeMPC_llb02, FireFlyOffsetFreeMPC_slb02, FireFlyOffsetFreeMPC_rilb02, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z02, FireFlyOffsetFreeMPC_ubIdx02, FireFlyOffsetFreeMPC_ub02, FireFlyOffsetFreeMPC_lub02, FireFlyOffsetFreeMPC_sub02, FireFlyOffsetFreeMPC_riub02, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb03, FireFlyOffsetFreeMPC_z03, FireFlyOffsetFreeMPC_lbIdx03, FireFlyOffsetFreeMPC_llb03, FireFlyOffsetFreeMPC_slb03, FireFlyOffsetFreeMPC_rilb03, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z03, FireFlyOffsetFreeMPC_ubIdx03, FireFlyOffsetFreeMPC_ub03, FireFlyOffsetFreeMPC_lub03, FireFlyOffsetFreeMPC_sub03, FireFlyOffsetFreeMPC_riub03, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb04, FireFlyOffsetFreeMPC_z04, FireFlyOffsetFreeMPC_lbIdx04, FireFlyOffsetFreeMPC_llb04, FireFlyOffsetFreeMPC_slb04, FireFlyOffsetFreeMPC_rilb04, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z04, FireFlyOffsetFreeMPC_ubIdx04, FireFlyOffsetFreeMPC_ub04, FireFlyOffsetFreeMPC_lub04, FireFlyOffsetFreeMPC_sub04, FireFlyOffsetFreeMPC_riub04, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb05, FireFlyOffsetFreeMPC_z05, FireFlyOffsetFreeMPC_lbIdx05, FireFlyOffsetFreeMPC_llb05, FireFlyOffsetFreeMPC_slb05, FireFlyOffsetFreeMPC_rilb05, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z05, FireFlyOffsetFreeMPC_ubIdx05, FireFlyOffsetFreeMPC_ub05, FireFlyOffsetFreeMPC_lub05, FireFlyOffsetFreeMPC_sub05, FireFlyOffsetFreeMPC_riub05, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb06, FireFlyOffsetFreeMPC_z06, FireFlyOffsetFreeMPC_lbIdx06, FireFlyOffsetFreeMPC_llb06, FireFlyOffsetFreeMPC_slb06, FireFlyOffsetFreeMPC_rilb06, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z06, FireFlyOffsetFreeMPC_ubIdx06, FireFlyOffsetFreeMPC_ub06, FireFlyOffsetFreeMPC_lub06, FireFlyOffsetFreeMPC_sub06, FireFlyOffsetFreeMPC_riub06, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb07, FireFlyOffsetFreeMPC_z07, FireFlyOffsetFreeMPC_lbIdx07, FireFlyOffsetFreeMPC_llb07, FireFlyOffsetFreeMPC_slb07, FireFlyOffsetFreeMPC_rilb07, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z07, FireFlyOffsetFreeMPC_ubIdx07, FireFlyOffsetFreeMPC_ub07, FireFlyOffsetFreeMPC_lub07, FireFlyOffsetFreeMPC_sub07, FireFlyOffsetFreeMPC_riub07, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb08, FireFlyOffsetFreeMPC_z08, FireFlyOffsetFreeMPC_lbIdx08, FireFlyOffsetFreeMPC_llb08, FireFlyOffsetFreeMPC_slb08, FireFlyOffsetFreeMPC_rilb08, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z08, FireFlyOffsetFreeMPC_ubIdx08, FireFlyOffsetFreeMPC_ub08, FireFlyOffsetFreeMPC_lub08, FireFlyOffsetFreeMPC_sub08, FireFlyOffsetFreeMPC_riub08, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb09, FireFlyOffsetFreeMPC_z09, FireFlyOffsetFreeMPC_lbIdx09, FireFlyOffsetFreeMPC_llb09, FireFlyOffsetFreeMPC_slb09, FireFlyOffsetFreeMPC_rilb09, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z09, FireFlyOffsetFreeMPC_ubIdx09, FireFlyOffsetFreeMPC_ub09, FireFlyOffsetFreeMPC_lub09, FireFlyOffsetFreeMPC_sub09, FireFlyOffsetFreeMPC_riub09, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb10, FireFlyOffsetFreeMPC_z10, FireFlyOffsetFreeMPC_lbIdx10, FireFlyOffsetFreeMPC_llb10, FireFlyOffsetFreeMPC_slb10, FireFlyOffsetFreeMPC_rilb10, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z10, FireFlyOffsetFreeMPC_ubIdx10, FireFlyOffsetFreeMPC_ub10, FireFlyOffsetFreeMPC_lub10, FireFlyOffsetFreeMPC_sub10, FireFlyOffsetFreeMPC_riub10, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb11, FireFlyOffsetFreeMPC_z11, FireFlyOffsetFreeMPC_lbIdx11, FireFlyOffsetFreeMPC_llb11, FireFlyOffsetFreeMPC_slb11, FireFlyOffsetFreeMPC_rilb11, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z11, FireFlyOffsetFreeMPC_ubIdx11, FireFlyOffsetFreeMPC_ub11, FireFlyOffsetFreeMPC_lub11, FireFlyOffsetFreeMPC_sub11, FireFlyOffsetFreeMPC_riub11, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb12, FireFlyOffsetFreeMPC_z12, FireFlyOffsetFreeMPC_lbIdx12, FireFlyOffsetFreeMPC_llb12, FireFlyOffsetFreeMPC_slb12, FireFlyOffsetFreeMPC_rilb12, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z12, FireFlyOffsetFreeMPC_ubIdx12, FireFlyOffsetFreeMPC_ub12, FireFlyOffsetFreeMPC_lub12, FireFlyOffsetFreeMPC_sub12, FireFlyOffsetFreeMPC_riub12, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb13, FireFlyOffsetFreeMPC_z13, FireFlyOffsetFreeMPC_lbIdx13, FireFlyOffsetFreeMPC_llb13, FireFlyOffsetFreeMPC_slb13, FireFlyOffsetFreeMPC_rilb13, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z13, FireFlyOffsetFreeMPC_ubIdx13, FireFlyOffsetFreeMPC_ub13, FireFlyOffsetFreeMPC_lub13, FireFlyOffsetFreeMPC_sub13, FireFlyOffsetFreeMPC_riub13, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb14, FireFlyOffsetFreeMPC_z14, FireFlyOffsetFreeMPC_lbIdx14, FireFlyOffsetFreeMPC_llb14, FireFlyOffsetFreeMPC_slb14, FireFlyOffsetFreeMPC_rilb14, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z14, FireFlyOffsetFreeMPC_ubIdx14, FireFlyOffsetFreeMPC_ub14, FireFlyOffsetFreeMPC_lub14, FireFlyOffsetFreeMPC_sub14, FireFlyOffsetFreeMPC_riub14, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb15, FireFlyOffsetFreeMPC_z15, FireFlyOffsetFreeMPC_lbIdx15, FireFlyOffsetFreeMPC_llb15, FireFlyOffsetFreeMPC_slb15, FireFlyOffsetFreeMPC_rilb15, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z15, FireFlyOffsetFreeMPC_ubIdx15, FireFlyOffsetFreeMPC_ub15, FireFlyOffsetFreeMPC_lub15, FireFlyOffsetFreeMPC_sub15, FireFlyOffsetFreeMPC_riub15, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb16, FireFlyOffsetFreeMPC_z16, FireFlyOffsetFreeMPC_lbIdx16, FireFlyOffsetFreeMPC_llb16, FireFlyOffsetFreeMPC_slb16, FireFlyOffsetFreeMPC_rilb16, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z16, FireFlyOffsetFreeMPC_ubIdx16, FireFlyOffsetFreeMPC_ub16, FireFlyOffsetFreeMPC_lub16, FireFlyOffsetFreeMPC_sub16, FireFlyOffsetFreeMPC_riub16, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb17, FireFlyOffsetFreeMPC_z17, FireFlyOffsetFreeMPC_lbIdx17, FireFlyOffsetFreeMPC_llb17, FireFlyOffsetFreeMPC_slb17, FireFlyOffsetFreeMPC_rilb17, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z17, FireFlyOffsetFreeMPC_ubIdx17, FireFlyOffsetFreeMPC_ub17, FireFlyOffsetFreeMPC_lub17, FireFlyOffsetFreeMPC_sub17, FireFlyOffsetFreeMPC_riub17, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_6(FireFlyOffsetFreeMPC_lb18, FireFlyOffsetFreeMPC_z18, FireFlyOffsetFreeMPC_lbIdx18, FireFlyOffsetFreeMPC_llb18, FireFlyOffsetFreeMPC_slb18, FireFlyOffsetFreeMPC_rilb18, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_6(FireFlyOffsetFreeMPC_z18, FireFlyOffsetFreeMPC_ubIdx18, FireFlyOffsetFreeMPC_ub18, FireFlyOffsetFreeMPC_lub18, FireFlyOffsetFreeMPC_sub18, FireFlyOffsetFreeMPC_riub18, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD3_3(FireFlyOffsetFreeMPC_lb19, FireFlyOffsetFreeMPC_z19, FireFlyOffsetFreeMPC_lbIdx19, FireFlyOffsetFreeMPC_llb19, FireFlyOffsetFreeMPC_slb19, FireFlyOffsetFreeMPC_rilb19, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_VSUBADD2_3(FireFlyOffsetFreeMPC_z19, FireFlyOffsetFreeMPC_ubIdx19, FireFlyOffsetFreeMPC_ub19, FireFlyOffsetFreeMPC_lub19, FireFlyOffsetFreeMPC_sub19, FireFlyOffsetFreeMPC_riub19, &info->dgap, &info->res_ineq);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub00, FireFlyOffsetFreeMPC_sub00, FireFlyOffsetFreeMPC_riub00, FireFlyOffsetFreeMPC_llb00, FireFlyOffsetFreeMPC_slb00, FireFlyOffsetFreeMPC_rilb00, FireFlyOffsetFreeMPC_lbIdx00, FireFlyOffsetFreeMPC_ubIdx00, FireFlyOffsetFreeMPC_grad_ineq00, FireFlyOffsetFreeMPC_lubbysub00, FireFlyOffsetFreeMPC_llbbyslb00);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub01, FireFlyOffsetFreeMPC_sub01, FireFlyOffsetFreeMPC_riub01, FireFlyOffsetFreeMPC_llb01, FireFlyOffsetFreeMPC_slb01, FireFlyOffsetFreeMPC_rilb01, FireFlyOffsetFreeMPC_lbIdx01, FireFlyOffsetFreeMPC_ubIdx01, FireFlyOffsetFreeMPC_grad_ineq01, FireFlyOffsetFreeMPC_lubbysub01, FireFlyOffsetFreeMPC_llbbyslb01);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub02, FireFlyOffsetFreeMPC_sub02, FireFlyOffsetFreeMPC_riub02, FireFlyOffsetFreeMPC_llb02, FireFlyOffsetFreeMPC_slb02, FireFlyOffsetFreeMPC_rilb02, FireFlyOffsetFreeMPC_lbIdx02, FireFlyOffsetFreeMPC_ubIdx02, FireFlyOffsetFreeMPC_grad_ineq02, FireFlyOffsetFreeMPC_lubbysub02, FireFlyOffsetFreeMPC_llbbyslb02);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub03, FireFlyOffsetFreeMPC_sub03, FireFlyOffsetFreeMPC_riub03, FireFlyOffsetFreeMPC_llb03, FireFlyOffsetFreeMPC_slb03, FireFlyOffsetFreeMPC_rilb03, FireFlyOffsetFreeMPC_lbIdx03, FireFlyOffsetFreeMPC_ubIdx03, FireFlyOffsetFreeMPC_grad_ineq03, FireFlyOffsetFreeMPC_lubbysub03, FireFlyOffsetFreeMPC_llbbyslb03);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub04, FireFlyOffsetFreeMPC_sub04, FireFlyOffsetFreeMPC_riub04, FireFlyOffsetFreeMPC_llb04, FireFlyOffsetFreeMPC_slb04, FireFlyOffsetFreeMPC_rilb04, FireFlyOffsetFreeMPC_lbIdx04, FireFlyOffsetFreeMPC_ubIdx04, FireFlyOffsetFreeMPC_grad_ineq04, FireFlyOffsetFreeMPC_lubbysub04, FireFlyOffsetFreeMPC_llbbyslb04);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub05, FireFlyOffsetFreeMPC_sub05, FireFlyOffsetFreeMPC_riub05, FireFlyOffsetFreeMPC_llb05, FireFlyOffsetFreeMPC_slb05, FireFlyOffsetFreeMPC_rilb05, FireFlyOffsetFreeMPC_lbIdx05, FireFlyOffsetFreeMPC_ubIdx05, FireFlyOffsetFreeMPC_grad_ineq05, FireFlyOffsetFreeMPC_lubbysub05, FireFlyOffsetFreeMPC_llbbyslb05);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub06, FireFlyOffsetFreeMPC_sub06, FireFlyOffsetFreeMPC_riub06, FireFlyOffsetFreeMPC_llb06, FireFlyOffsetFreeMPC_slb06, FireFlyOffsetFreeMPC_rilb06, FireFlyOffsetFreeMPC_lbIdx06, FireFlyOffsetFreeMPC_ubIdx06, FireFlyOffsetFreeMPC_grad_ineq06, FireFlyOffsetFreeMPC_lubbysub06, FireFlyOffsetFreeMPC_llbbyslb06);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub07, FireFlyOffsetFreeMPC_sub07, FireFlyOffsetFreeMPC_riub07, FireFlyOffsetFreeMPC_llb07, FireFlyOffsetFreeMPC_slb07, FireFlyOffsetFreeMPC_rilb07, FireFlyOffsetFreeMPC_lbIdx07, FireFlyOffsetFreeMPC_ubIdx07, FireFlyOffsetFreeMPC_grad_ineq07, FireFlyOffsetFreeMPC_lubbysub07, FireFlyOffsetFreeMPC_llbbyslb07);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub08, FireFlyOffsetFreeMPC_sub08, FireFlyOffsetFreeMPC_riub08, FireFlyOffsetFreeMPC_llb08, FireFlyOffsetFreeMPC_slb08, FireFlyOffsetFreeMPC_rilb08, FireFlyOffsetFreeMPC_lbIdx08, FireFlyOffsetFreeMPC_ubIdx08, FireFlyOffsetFreeMPC_grad_ineq08, FireFlyOffsetFreeMPC_lubbysub08, FireFlyOffsetFreeMPC_llbbyslb08);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub09, FireFlyOffsetFreeMPC_sub09, FireFlyOffsetFreeMPC_riub09, FireFlyOffsetFreeMPC_llb09, FireFlyOffsetFreeMPC_slb09, FireFlyOffsetFreeMPC_rilb09, FireFlyOffsetFreeMPC_lbIdx09, FireFlyOffsetFreeMPC_ubIdx09, FireFlyOffsetFreeMPC_grad_ineq09, FireFlyOffsetFreeMPC_lubbysub09, FireFlyOffsetFreeMPC_llbbyslb09);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub10, FireFlyOffsetFreeMPC_sub10, FireFlyOffsetFreeMPC_riub10, FireFlyOffsetFreeMPC_llb10, FireFlyOffsetFreeMPC_slb10, FireFlyOffsetFreeMPC_rilb10, FireFlyOffsetFreeMPC_lbIdx10, FireFlyOffsetFreeMPC_ubIdx10, FireFlyOffsetFreeMPC_grad_ineq10, FireFlyOffsetFreeMPC_lubbysub10, FireFlyOffsetFreeMPC_llbbyslb10);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub11, FireFlyOffsetFreeMPC_sub11, FireFlyOffsetFreeMPC_riub11, FireFlyOffsetFreeMPC_llb11, FireFlyOffsetFreeMPC_slb11, FireFlyOffsetFreeMPC_rilb11, FireFlyOffsetFreeMPC_lbIdx11, FireFlyOffsetFreeMPC_ubIdx11, FireFlyOffsetFreeMPC_grad_ineq11, FireFlyOffsetFreeMPC_lubbysub11, FireFlyOffsetFreeMPC_llbbyslb11);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub12, FireFlyOffsetFreeMPC_sub12, FireFlyOffsetFreeMPC_riub12, FireFlyOffsetFreeMPC_llb12, FireFlyOffsetFreeMPC_slb12, FireFlyOffsetFreeMPC_rilb12, FireFlyOffsetFreeMPC_lbIdx12, FireFlyOffsetFreeMPC_ubIdx12, FireFlyOffsetFreeMPC_grad_ineq12, FireFlyOffsetFreeMPC_lubbysub12, FireFlyOffsetFreeMPC_llbbyslb12);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub13, FireFlyOffsetFreeMPC_sub13, FireFlyOffsetFreeMPC_riub13, FireFlyOffsetFreeMPC_llb13, FireFlyOffsetFreeMPC_slb13, FireFlyOffsetFreeMPC_rilb13, FireFlyOffsetFreeMPC_lbIdx13, FireFlyOffsetFreeMPC_ubIdx13, FireFlyOffsetFreeMPC_grad_ineq13, FireFlyOffsetFreeMPC_lubbysub13, FireFlyOffsetFreeMPC_llbbyslb13);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub14, FireFlyOffsetFreeMPC_sub14, FireFlyOffsetFreeMPC_riub14, FireFlyOffsetFreeMPC_llb14, FireFlyOffsetFreeMPC_slb14, FireFlyOffsetFreeMPC_rilb14, FireFlyOffsetFreeMPC_lbIdx14, FireFlyOffsetFreeMPC_ubIdx14, FireFlyOffsetFreeMPC_grad_ineq14, FireFlyOffsetFreeMPC_lubbysub14, FireFlyOffsetFreeMPC_llbbyslb14);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub15, FireFlyOffsetFreeMPC_sub15, FireFlyOffsetFreeMPC_riub15, FireFlyOffsetFreeMPC_llb15, FireFlyOffsetFreeMPC_slb15, FireFlyOffsetFreeMPC_rilb15, FireFlyOffsetFreeMPC_lbIdx15, FireFlyOffsetFreeMPC_ubIdx15, FireFlyOffsetFreeMPC_grad_ineq15, FireFlyOffsetFreeMPC_lubbysub15, FireFlyOffsetFreeMPC_llbbyslb15);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub16, FireFlyOffsetFreeMPC_sub16, FireFlyOffsetFreeMPC_riub16, FireFlyOffsetFreeMPC_llb16, FireFlyOffsetFreeMPC_slb16, FireFlyOffsetFreeMPC_rilb16, FireFlyOffsetFreeMPC_lbIdx16, FireFlyOffsetFreeMPC_ubIdx16, FireFlyOffsetFreeMPC_grad_ineq16, FireFlyOffsetFreeMPC_lubbysub16, FireFlyOffsetFreeMPC_llbbyslb16);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub17, FireFlyOffsetFreeMPC_sub17, FireFlyOffsetFreeMPC_riub17, FireFlyOffsetFreeMPC_llb17, FireFlyOffsetFreeMPC_slb17, FireFlyOffsetFreeMPC_rilb17, FireFlyOffsetFreeMPC_lbIdx17, FireFlyOffsetFreeMPC_ubIdx17, FireFlyOffsetFreeMPC_grad_ineq17, FireFlyOffsetFreeMPC_lubbysub17, FireFlyOffsetFreeMPC_llbbyslb17);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_17_6_6(FireFlyOffsetFreeMPC_lub18, FireFlyOffsetFreeMPC_sub18, FireFlyOffsetFreeMPC_riub18, FireFlyOffsetFreeMPC_llb18, FireFlyOffsetFreeMPC_slb18, FireFlyOffsetFreeMPC_rilb18, FireFlyOffsetFreeMPC_lbIdx18, FireFlyOffsetFreeMPC_ubIdx18, FireFlyOffsetFreeMPC_grad_ineq18, FireFlyOffsetFreeMPC_lubbysub18, FireFlyOffsetFreeMPC_llbbyslb18);
FireFlyOffsetFreeMPC_LA_INEQ_B_GRAD_14_3_3(FireFlyOffsetFreeMPC_lub19, FireFlyOffsetFreeMPC_sub19, FireFlyOffsetFreeMPC_riub19, FireFlyOffsetFreeMPC_llb19, FireFlyOffsetFreeMPC_slb19, FireFlyOffsetFreeMPC_rilb19, FireFlyOffsetFreeMPC_lbIdx19, FireFlyOffsetFreeMPC_ubIdx19, FireFlyOffsetFreeMPC_grad_ineq19, FireFlyOffsetFreeMPC_lubbysub19, FireFlyOffsetFreeMPC_llbbyslb19);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
if( info->mu < FireFlyOffsetFreeMPC_SET_ACC_KKTCOMPL
    && (info->rdgap < FireFlyOffsetFreeMPC_SET_ACC_RDGAP || info->dgap < FireFlyOffsetFreeMPC_SET_ACC_KKTCOMPL)
    && info->res_eq < FireFlyOffsetFreeMPC_SET_ACC_RESEQ
    && info->res_ineq < FireFlyOffsetFreeMPC_SET_ACC_RESINEQ ){
exitcode = FireFlyOffsetFreeMPC_OPTIMAL; break; }
if( info->it == FireFlyOffsetFreeMPC_SET_MAXIT ){
exitcode = FireFlyOffsetFreeMPC_MAXITREACHED; break; }
FireFlyOffsetFreeMPC_LA_VVADD3_337(FireFlyOffsetFreeMPC_grad_cost, FireFlyOffsetFreeMPC_grad_eq, FireFlyOffsetFreeMPC_grad_ineq, FireFlyOffsetFreeMPC_rd);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb00, FireFlyOffsetFreeMPC_lbIdx00, FireFlyOffsetFreeMPC_lubbysub00, FireFlyOffsetFreeMPC_ubIdx00, FireFlyOffsetFreeMPC_Phi00);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi00);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_28_17(FireFlyOffsetFreeMPC_Phi00, FireFlyOffsetFreeMPC_C00, FireFlyOffsetFreeMPC_V00);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi00, FireFlyOffsetFreeMPC_rd00, FireFlyOffsetFreeMPC_Lbyrd00);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb01, FireFlyOffsetFreeMPC_lbIdx01, FireFlyOffsetFreeMPC_lubbysub01, FireFlyOffsetFreeMPC_ubIdx01, FireFlyOffsetFreeMPC_Phi01);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi01);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi01, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V01);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_28_17(FireFlyOffsetFreeMPC_Phi01, FireFlyOffsetFreeMPC_D01, FireFlyOffsetFreeMPC_W01);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_28_17_14(FireFlyOffsetFreeMPC_W01, FireFlyOffsetFreeMPC_V01, FireFlyOffsetFreeMPC_Ysd01);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi01, FireFlyOffsetFreeMPC_rd01, FireFlyOffsetFreeMPC_Lbyrd01);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb02, FireFlyOffsetFreeMPC_lbIdx02, FireFlyOffsetFreeMPC_lubbysub02, FireFlyOffsetFreeMPC_ubIdx02, FireFlyOffsetFreeMPC_Phi02);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi02);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi02, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V02);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi02, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W02);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W02, FireFlyOffsetFreeMPC_V02, FireFlyOffsetFreeMPC_Ysd02);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi02, FireFlyOffsetFreeMPC_rd02, FireFlyOffsetFreeMPC_Lbyrd02);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb03, FireFlyOffsetFreeMPC_lbIdx03, FireFlyOffsetFreeMPC_lubbysub03, FireFlyOffsetFreeMPC_ubIdx03, FireFlyOffsetFreeMPC_Phi03);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi03);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi03, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V03);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi03, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W03);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W03, FireFlyOffsetFreeMPC_V03, FireFlyOffsetFreeMPC_Ysd03);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi03, FireFlyOffsetFreeMPC_rd03, FireFlyOffsetFreeMPC_Lbyrd03);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb04, FireFlyOffsetFreeMPC_lbIdx04, FireFlyOffsetFreeMPC_lubbysub04, FireFlyOffsetFreeMPC_ubIdx04, FireFlyOffsetFreeMPC_Phi04);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi04);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi04, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V04);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi04, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W04);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W04, FireFlyOffsetFreeMPC_V04, FireFlyOffsetFreeMPC_Ysd04);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi04, FireFlyOffsetFreeMPC_rd04, FireFlyOffsetFreeMPC_Lbyrd04);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb05, FireFlyOffsetFreeMPC_lbIdx05, FireFlyOffsetFreeMPC_lubbysub05, FireFlyOffsetFreeMPC_ubIdx05, FireFlyOffsetFreeMPC_Phi05);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi05);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi05, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V05);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi05, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W05);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W05, FireFlyOffsetFreeMPC_V05, FireFlyOffsetFreeMPC_Ysd05);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi05, FireFlyOffsetFreeMPC_rd05, FireFlyOffsetFreeMPC_Lbyrd05);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb06, FireFlyOffsetFreeMPC_lbIdx06, FireFlyOffsetFreeMPC_lubbysub06, FireFlyOffsetFreeMPC_ubIdx06, FireFlyOffsetFreeMPC_Phi06);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi06);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi06, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V06);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi06, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W06);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W06, FireFlyOffsetFreeMPC_V06, FireFlyOffsetFreeMPC_Ysd06);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi06, FireFlyOffsetFreeMPC_rd06, FireFlyOffsetFreeMPC_Lbyrd06);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb07, FireFlyOffsetFreeMPC_lbIdx07, FireFlyOffsetFreeMPC_lubbysub07, FireFlyOffsetFreeMPC_ubIdx07, FireFlyOffsetFreeMPC_Phi07);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi07);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi07, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V07);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi07, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W07);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W07, FireFlyOffsetFreeMPC_V07, FireFlyOffsetFreeMPC_Ysd07);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi07, FireFlyOffsetFreeMPC_rd07, FireFlyOffsetFreeMPC_Lbyrd07);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb08, FireFlyOffsetFreeMPC_lbIdx08, FireFlyOffsetFreeMPC_lubbysub08, FireFlyOffsetFreeMPC_ubIdx08, FireFlyOffsetFreeMPC_Phi08);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi08);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi08, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V08);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi08, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W08);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W08, FireFlyOffsetFreeMPC_V08, FireFlyOffsetFreeMPC_Ysd08);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi08, FireFlyOffsetFreeMPC_rd08, FireFlyOffsetFreeMPC_Lbyrd08);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb09, FireFlyOffsetFreeMPC_lbIdx09, FireFlyOffsetFreeMPC_lubbysub09, FireFlyOffsetFreeMPC_ubIdx09, FireFlyOffsetFreeMPC_Phi09);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi09);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi09, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V09);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi09, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W09);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W09, FireFlyOffsetFreeMPC_V09, FireFlyOffsetFreeMPC_Ysd09);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi09, FireFlyOffsetFreeMPC_rd09, FireFlyOffsetFreeMPC_Lbyrd09);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb10, FireFlyOffsetFreeMPC_lbIdx10, FireFlyOffsetFreeMPC_lubbysub10, FireFlyOffsetFreeMPC_ubIdx10, FireFlyOffsetFreeMPC_Phi10);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi10);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi10, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V10);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi10, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W10);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W10, FireFlyOffsetFreeMPC_V10, FireFlyOffsetFreeMPC_Ysd10);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi10, FireFlyOffsetFreeMPC_rd10, FireFlyOffsetFreeMPC_Lbyrd10);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb11, FireFlyOffsetFreeMPC_lbIdx11, FireFlyOffsetFreeMPC_lubbysub11, FireFlyOffsetFreeMPC_ubIdx11, FireFlyOffsetFreeMPC_Phi11);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi11);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi11, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V11);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi11, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W11);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W11, FireFlyOffsetFreeMPC_V11, FireFlyOffsetFreeMPC_Ysd11);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi11, FireFlyOffsetFreeMPC_rd11, FireFlyOffsetFreeMPC_Lbyrd11);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb12, FireFlyOffsetFreeMPC_lbIdx12, FireFlyOffsetFreeMPC_lubbysub12, FireFlyOffsetFreeMPC_ubIdx12, FireFlyOffsetFreeMPC_Phi12);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi12);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi12, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V12);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi12, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W12);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W12, FireFlyOffsetFreeMPC_V12, FireFlyOffsetFreeMPC_Ysd12);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi12, FireFlyOffsetFreeMPC_rd12, FireFlyOffsetFreeMPC_Lbyrd12);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb13, FireFlyOffsetFreeMPC_lbIdx13, FireFlyOffsetFreeMPC_lubbysub13, FireFlyOffsetFreeMPC_ubIdx13, FireFlyOffsetFreeMPC_Phi13);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi13);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi13, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V13);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi13, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W13);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W13, FireFlyOffsetFreeMPC_V13, FireFlyOffsetFreeMPC_Ysd13);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi13, FireFlyOffsetFreeMPC_rd13, FireFlyOffsetFreeMPC_Lbyrd13);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb14, FireFlyOffsetFreeMPC_lbIdx14, FireFlyOffsetFreeMPC_lubbysub14, FireFlyOffsetFreeMPC_ubIdx14, FireFlyOffsetFreeMPC_Phi14);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi14);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi14, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V14);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi14, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W14);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W14, FireFlyOffsetFreeMPC_V14, FireFlyOffsetFreeMPC_Ysd14);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi14, FireFlyOffsetFreeMPC_rd14, FireFlyOffsetFreeMPC_Lbyrd14);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb15, FireFlyOffsetFreeMPC_lbIdx15, FireFlyOffsetFreeMPC_lubbysub15, FireFlyOffsetFreeMPC_ubIdx15, FireFlyOffsetFreeMPC_Phi15);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi15);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi15, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V15);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi15, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W15);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W15, FireFlyOffsetFreeMPC_V15, FireFlyOffsetFreeMPC_Ysd15);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi15, FireFlyOffsetFreeMPC_rd15, FireFlyOffsetFreeMPC_Lbyrd15);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb16, FireFlyOffsetFreeMPC_lbIdx16, FireFlyOffsetFreeMPC_lubbysub16, FireFlyOffsetFreeMPC_ubIdx16, FireFlyOffsetFreeMPC_Phi16);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi16);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi16, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V16);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi16, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W16);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W16, FireFlyOffsetFreeMPC_V16, FireFlyOffsetFreeMPC_Ysd16);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi16, FireFlyOffsetFreeMPC_rd16, FireFlyOffsetFreeMPC_Lbyrd16);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb17, FireFlyOffsetFreeMPC_lbIdx17, FireFlyOffsetFreeMPC_lubbysub17, FireFlyOffsetFreeMPC_ubIdx17, FireFlyOffsetFreeMPC_Phi17);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi17);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi17, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V17);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi17, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W17);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W17, FireFlyOffsetFreeMPC_V17, FireFlyOffsetFreeMPC_Ysd17);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi17, FireFlyOffsetFreeMPC_rd17, FireFlyOffsetFreeMPC_Lbyrd17);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_17_6_6(params->H, FireFlyOffsetFreeMPC_llbbyslb18, FireFlyOffsetFreeMPC_lbIdx18, FireFlyOffsetFreeMPC_lubbysub18, FireFlyOffsetFreeMPC_ubIdx18, FireFlyOffsetFreeMPC_Phi18);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_17(FireFlyOffsetFreeMPC_Phi18);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi18, FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_V18);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_17(FireFlyOffsetFreeMPC_Phi18, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_W18);
FireFlyOffsetFreeMPC_LA_DENSE_MMTM_14_17_14(FireFlyOffsetFreeMPC_W18, FireFlyOffsetFreeMPC_V18, FireFlyOffsetFreeMPC_Ysd18);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi18, FireFlyOffsetFreeMPC_rd18, FireFlyOffsetFreeMPC_Lbyrd18);
FireFlyOffsetFreeMPC_LA_INEQ_DENSE_HESS_14_3_3(params->H_N, FireFlyOffsetFreeMPC_llbbyslb19, FireFlyOffsetFreeMPC_lbIdx19, FireFlyOffsetFreeMPC_lubbysub19, FireFlyOffsetFreeMPC_ubIdx19, FireFlyOffsetFreeMPC_Phi19);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL2_14(FireFlyOffsetFreeMPC_Phi19);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Phi19, FireFlyOffsetFreeMPC_D19, FireFlyOffsetFreeMPC_W19);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Phi19, FireFlyOffsetFreeMPC_rd19, FireFlyOffsetFreeMPC_Lbyrd19);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_28_17_17(FireFlyOffsetFreeMPC_V00, FireFlyOffsetFreeMPC_W01, FireFlyOffsetFreeMPC_Yd00);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_28_17_17(FireFlyOffsetFreeMPC_V00, FireFlyOffsetFreeMPC_Lbyrd00, FireFlyOffsetFreeMPC_W01, FireFlyOffsetFreeMPC_Lbyrd01, FireFlyOffsetFreeMPC_re00, FireFlyOffsetFreeMPC_beta00);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V01, FireFlyOffsetFreeMPC_W02, FireFlyOffsetFreeMPC_Yd01);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V01, FireFlyOffsetFreeMPC_Lbyrd01, FireFlyOffsetFreeMPC_W02, FireFlyOffsetFreeMPC_Lbyrd02, FireFlyOffsetFreeMPC_re01, FireFlyOffsetFreeMPC_beta01);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V02, FireFlyOffsetFreeMPC_W03, FireFlyOffsetFreeMPC_Yd02);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V02, FireFlyOffsetFreeMPC_Lbyrd02, FireFlyOffsetFreeMPC_W03, FireFlyOffsetFreeMPC_Lbyrd03, FireFlyOffsetFreeMPC_re02, FireFlyOffsetFreeMPC_beta02);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V03, FireFlyOffsetFreeMPC_W04, FireFlyOffsetFreeMPC_Yd03);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V03, FireFlyOffsetFreeMPC_Lbyrd03, FireFlyOffsetFreeMPC_W04, FireFlyOffsetFreeMPC_Lbyrd04, FireFlyOffsetFreeMPC_re03, FireFlyOffsetFreeMPC_beta03);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V04, FireFlyOffsetFreeMPC_W05, FireFlyOffsetFreeMPC_Yd04);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V04, FireFlyOffsetFreeMPC_Lbyrd04, FireFlyOffsetFreeMPC_W05, FireFlyOffsetFreeMPC_Lbyrd05, FireFlyOffsetFreeMPC_re04, FireFlyOffsetFreeMPC_beta04);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V05, FireFlyOffsetFreeMPC_W06, FireFlyOffsetFreeMPC_Yd05);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V05, FireFlyOffsetFreeMPC_Lbyrd05, FireFlyOffsetFreeMPC_W06, FireFlyOffsetFreeMPC_Lbyrd06, FireFlyOffsetFreeMPC_re05, FireFlyOffsetFreeMPC_beta05);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V06, FireFlyOffsetFreeMPC_W07, FireFlyOffsetFreeMPC_Yd06);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V06, FireFlyOffsetFreeMPC_Lbyrd06, FireFlyOffsetFreeMPC_W07, FireFlyOffsetFreeMPC_Lbyrd07, FireFlyOffsetFreeMPC_re06, FireFlyOffsetFreeMPC_beta06);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V07, FireFlyOffsetFreeMPC_W08, FireFlyOffsetFreeMPC_Yd07);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V07, FireFlyOffsetFreeMPC_Lbyrd07, FireFlyOffsetFreeMPC_W08, FireFlyOffsetFreeMPC_Lbyrd08, FireFlyOffsetFreeMPC_re07, FireFlyOffsetFreeMPC_beta07);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V08, FireFlyOffsetFreeMPC_W09, FireFlyOffsetFreeMPC_Yd08);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V08, FireFlyOffsetFreeMPC_Lbyrd08, FireFlyOffsetFreeMPC_W09, FireFlyOffsetFreeMPC_Lbyrd09, FireFlyOffsetFreeMPC_re08, FireFlyOffsetFreeMPC_beta08);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V09, FireFlyOffsetFreeMPC_W10, FireFlyOffsetFreeMPC_Yd09);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V09, FireFlyOffsetFreeMPC_Lbyrd09, FireFlyOffsetFreeMPC_W10, FireFlyOffsetFreeMPC_Lbyrd10, FireFlyOffsetFreeMPC_re09, FireFlyOffsetFreeMPC_beta09);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V10, FireFlyOffsetFreeMPC_W11, FireFlyOffsetFreeMPC_Yd10);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V10, FireFlyOffsetFreeMPC_Lbyrd10, FireFlyOffsetFreeMPC_W11, FireFlyOffsetFreeMPC_Lbyrd11, FireFlyOffsetFreeMPC_re10, FireFlyOffsetFreeMPC_beta10);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V11, FireFlyOffsetFreeMPC_W12, FireFlyOffsetFreeMPC_Yd11);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V11, FireFlyOffsetFreeMPC_Lbyrd11, FireFlyOffsetFreeMPC_W12, FireFlyOffsetFreeMPC_Lbyrd12, FireFlyOffsetFreeMPC_re11, FireFlyOffsetFreeMPC_beta11);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V12, FireFlyOffsetFreeMPC_W13, FireFlyOffsetFreeMPC_Yd12);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V12, FireFlyOffsetFreeMPC_Lbyrd12, FireFlyOffsetFreeMPC_W13, FireFlyOffsetFreeMPC_Lbyrd13, FireFlyOffsetFreeMPC_re12, FireFlyOffsetFreeMPC_beta12);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V13, FireFlyOffsetFreeMPC_W14, FireFlyOffsetFreeMPC_Yd13);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V13, FireFlyOffsetFreeMPC_Lbyrd13, FireFlyOffsetFreeMPC_W14, FireFlyOffsetFreeMPC_Lbyrd14, FireFlyOffsetFreeMPC_re13, FireFlyOffsetFreeMPC_beta13);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V14, FireFlyOffsetFreeMPC_W15, FireFlyOffsetFreeMPC_Yd14);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V14, FireFlyOffsetFreeMPC_Lbyrd14, FireFlyOffsetFreeMPC_W15, FireFlyOffsetFreeMPC_Lbyrd15, FireFlyOffsetFreeMPC_re14, FireFlyOffsetFreeMPC_beta14);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V15, FireFlyOffsetFreeMPC_W16, FireFlyOffsetFreeMPC_Yd15);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V15, FireFlyOffsetFreeMPC_Lbyrd15, FireFlyOffsetFreeMPC_W16, FireFlyOffsetFreeMPC_Lbyrd16, FireFlyOffsetFreeMPC_re15, FireFlyOffsetFreeMPC_beta15);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V16, FireFlyOffsetFreeMPC_W17, FireFlyOffsetFreeMPC_Yd16);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V16, FireFlyOffsetFreeMPC_Lbyrd16, FireFlyOffsetFreeMPC_W17, FireFlyOffsetFreeMPC_Lbyrd17, FireFlyOffsetFreeMPC_re16, FireFlyOffsetFreeMPC_beta16);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_17(FireFlyOffsetFreeMPC_V17, FireFlyOffsetFreeMPC_W18, FireFlyOffsetFreeMPC_Yd17);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_17(FireFlyOffsetFreeMPC_V17, FireFlyOffsetFreeMPC_Lbyrd17, FireFlyOffsetFreeMPC_W18, FireFlyOffsetFreeMPC_Lbyrd18, FireFlyOffsetFreeMPC_re17, FireFlyOffsetFreeMPC_beta17);
FireFlyOffsetFreeMPC_LA_DENSE_MMT2_14_17_14(FireFlyOffsetFreeMPC_V18, FireFlyOffsetFreeMPC_W19, FireFlyOffsetFreeMPC_Yd18);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB2_14_17_14(FireFlyOffsetFreeMPC_V18, FireFlyOffsetFreeMPC_Lbyrd18, FireFlyOffsetFreeMPC_W19, FireFlyOffsetFreeMPC_Lbyrd19, FireFlyOffsetFreeMPC_re18, FireFlyOffsetFreeMPC_beta18);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_28(FireFlyOffsetFreeMPC_Yd00, FireFlyOffsetFreeMPC_Ld00);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_28(FireFlyOffsetFreeMPC_Ld00, FireFlyOffsetFreeMPC_beta00, FireFlyOffsetFreeMPC_yy00);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_28(FireFlyOffsetFreeMPC_Ld00, FireFlyOffsetFreeMPC_Ysd01, FireFlyOffsetFreeMPC_Lsd01);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_28(FireFlyOffsetFreeMPC_Lsd01, FireFlyOffsetFreeMPC_Yd01);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd01, FireFlyOffsetFreeMPC_Ld01);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_28(FireFlyOffsetFreeMPC_Lsd01, FireFlyOffsetFreeMPC_yy00, FireFlyOffsetFreeMPC_beta01, FireFlyOffsetFreeMPC_bmy01);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld01, FireFlyOffsetFreeMPC_bmy01, FireFlyOffsetFreeMPC_yy01);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld01, FireFlyOffsetFreeMPC_Ysd02, FireFlyOffsetFreeMPC_Lsd02);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd02, FireFlyOffsetFreeMPC_Yd02);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd02, FireFlyOffsetFreeMPC_Ld02);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd02, FireFlyOffsetFreeMPC_yy01, FireFlyOffsetFreeMPC_beta02, FireFlyOffsetFreeMPC_bmy02);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld02, FireFlyOffsetFreeMPC_bmy02, FireFlyOffsetFreeMPC_yy02);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld02, FireFlyOffsetFreeMPC_Ysd03, FireFlyOffsetFreeMPC_Lsd03);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd03, FireFlyOffsetFreeMPC_Yd03);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd03, FireFlyOffsetFreeMPC_Ld03);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd03, FireFlyOffsetFreeMPC_yy02, FireFlyOffsetFreeMPC_beta03, FireFlyOffsetFreeMPC_bmy03);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld03, FireFlyOffsetFreeMPC_bmy03, FireFlyOffsetFreeMPC_yy03);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld03, FireFlyOffsetFreeMPC_Ysd04, FireFlyOffsetFreeMPC_Lsd04);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd04, FireFlyOffsetFreeMPC_Yd04);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd04, FireFlyOffsetFreeMPC_Ld04);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd04, FireFlyOffsetFreeMPC_yy03, FireFlyOffsetFreeMPC_beta04, FireFlyOffsetFreeMPC_bmy04);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld04, FireFlyOffsetFreeMPC_bmy04, FireFlyOffsetFreeMPC_yy04);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld04, FireFlyOffsetFreeMPC_Ysd05, FireFlyOffsetFreeMPC_Lsd05);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd05, FireFlyOffsetFreeMPC_Yd05);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd05, FireFlyOffsetFreeMPC_Ld05);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd05, FireFlyOffsetFreeMPC_yy04, FireFlyOffsetFreeMPC_beta05, FireFlyOffsetFreeMPC_bmy05);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld05, FireFlyOffsetFreeMPC_bmy05, FireFlyOffsetFreeMPC_yy05);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld05, FireFlyOffsetFreeMPC_Ysd06, FireFlyOffsetFreeMPC_Lsd06);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd06, FireFlyOffsetFreeMPC_Yd06);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd06, FireFlyOffsetFreeMPC_Ld06);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd06, FireFlyOffsetFreeMPC_yy05, FireFlyOffsetFreeMPC_beta06, FireFlyOffsetFreeMPC_bmy06);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld06, FireFlyOffsetFreeMPC_bmy06, FireFlyOffsetFreeMPC_yy06);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld06, FireFlyOffsetFreeMPC_Ysd07, FireFlyOffsetFreeMPC_Lsd07);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd07, FireFlyOffsetFreeMPC_Yd07);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd07, FireFlyOffsetFreeMPC_Ld07);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd07, FireFlyOffsetFreeMPC_yy06, FireFlyOffsetFreeMPC_beta07, FireFlyOffsetFreeMPC_bmy07);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld07, FireFlyOffsetFreeMPC_bmy07, FireFlyOffsetFreeMPC_yy07);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld07, FireFlyOffsetFreeMPC_Ysd08, FireFlyOffsetFreeMPC_Lsd08);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd08, FireFlyOffsetFreeMPC_Yd08);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd08, FireFlyOffsetFreeMPC_Ld08);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd08, FireFlyOffsetFreeMPC_yy07, FireFlyOffsetFreeMPC_beta08, FireFlyOffsetFreeMPC_bmy08);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld08, FireFlyOffsetFreeMPC_bmy08, FireFlyOffsetFreeMPC_yy08);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld08, FireFlyOffsetFreeMPC_Ysd09, FireFlyOffsetFreeMPC_Lsd09);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd09, FireFlyOffsetFreeMPC_Yd09);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd09, FireFlyOffsetFreeMPC_Ld09);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd09, FireFlyOffsetFreeMPC_yy08, FireFlyOffsetFreeMPC_beta09, FireFlyOffsetFreeMPC_bmy09);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld09, FireFlyOffsetFreeMPC_bmy09, FireFlyOffsetFreeMPC_yy09);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld09, FireFlyOffsetFreeMPC_Ysd10, FireFlyOffsetFreeMPC_Lsd10);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd10, FireFlyOffsetFreeMPC_Yd10);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd10, FireFlyOffsetFreeMPC_Ld10);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd10, FireFlyOffsetFreeMPC_yy09, FireFlyOffsetFreeMPC_beta10, FireFlyOffsetFreeMPC_bmy10);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld10, FireFlyOffsetFreeMPC_bmy10, FireFlyOffsetFreeMPC_yy10);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld10, FireFlyOffsetFreeMPC_Ysd11, FireFlyOffsetFreeMPC_Lsd11);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd11, FireFlyOffsetFreeMPC_Yd11);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd11, FireFlyOffsetFreeMPC_Ld11);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd11, FireFlyOffsetFreeMPC_yy10, FireFlyOffsetFreeMPC_beta11, FireFlyOffsetFreeMPC_bmy11);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld11, FireFlyOffsetFreeMPC_bmy11, FireFlyOffsetFreeMPC_yy11);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld11, FireFlyOffsetFreeMPC_Ysd12, FireFlyOffsetFreeMPC_Lsd12);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd12, FireFlyOffsetFreeMPC_Yd12);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd12, FireFlyOffsetFreeMPC_Ld12);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd12, FireFlyOffsetFreeMPC_yy11, FireFlyOffsetFreeMPC_beta12, FireFlyOffsetFreeMPC_bmy12);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld12, FireFlyOffsetFreeMPC_bmy12, FireFlyOffsetFreeMPC_yy12);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld12, FireFlyOffsetFreeMPC_Ysd13, FireFlyOffsetFreeMPC_Lsd13);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd13, FireFlyOffsetFreeMPC_Yd13);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd13, FireFlyOffsetFreeMPC_Ld13);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd13, FireFlyOffsetFreeMPC_yy12, FireFlyOffsetFreeMPC_beta13, FireFlyOffsetFreeMPC_bmy13);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld13, FireFlyOffsetFreeMPC_bmy13, FireFlyOffsetFreeMPC_yy13);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld13, FireFlyOffsetFreeMPC_Ysd14, FireFlyOffsetFreeMPC_Lsd14);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd14, FireFlyOffsetFreeMPC_Yd14);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd14, FireFlyOffsetFreeMPC_Ld14);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd14, FireFlyOffsetFreeMPC_yy13, FireFlyOffsetFreeMPC_beta14, FireFlyOffsetFreeMPC_bmy14);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld14, FireFlyOffsetFreeMPC_bmy14, FireFlyOffsetFreeMPC_yy14);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld14, FireFlyOffsetFreeMPC_Ysd15, FireFlyOffsetFreeMPC_Lsd15);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd15, FireFlyOffsetFreeMPC_Yd15);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd15, FireFlyOffsetFreeMPC_Ld15);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd15, FireFlyOffsetFreeMPC_yy14, FireFlyOffsetFreeMPC_beta15, FireFlyOffsetFreeMPC_bmy15);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld15, FireFlyOffsetFreeMPC_bmy15, FireFlyOffsetFreeMPC_yy15);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld15, FireFlyOffsetFreeMPC_Ysd16, FireFlyOffsetFreeMPC_Lsd16);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd16, FireFlyOffsetFreeMPC_Yd16);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd16, FireFlyOffsetFreeMPC_Ld16);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd16, FireFlyOffsetFreeMPC_yy15, FireFlyOffsetFreeMPC_beta16, FireFlyOffsetFreeMPC_bmy16);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld16, FireFlyOffsetFreeMPC_bmy16, FireFlyOffsetFreeMPC_yy16);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld16, FireFlyOffsetFreeMPC_Ysd17, FireFlyOffsetFreeMPC_Lsd17);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd17, FireFlyOffsetFreeMPC_Yd17);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd17, FireFlyOffsetFreeMPC_Ld17);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd17, FireFlyOffsetFreeMPC_yy16, FireFlyOffsetFreeMPC_beta17, FireFlyOffsetFreeMPC_bmy17);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld17, FireFlyOffsetFreeMPC_bmy17, FireFlyOffsetFreeMPC_yy17);
FireFlyOffsetFreeMPC_LA_DENSE_MATRIXTFORWARDSUB_14_14(FireFlyOffsetFreeMPC_Ld17, FireFlyOffsetFreeMPC_Ysd18, FireFlyOffsetFreeMPC_Lsd18);
FireFlyOffsetFreeMPC_LA_DENSE_MMTSUB_14_14(FireFlyOffsetFreeMPC_Lsd18, FireFlyOffsetFreeMPC_Yd18);
FireFlyOffsetFreeMPC_LA_DENSE_CHOL_14(FireFlyOffsetFreeMPC_Yd18, FireFlyOffsetFreeMPC_Ld18);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd18, FireFlyOffsetFreeMPC_yy17, FireFlyOffsetFreeMPC_beta18, FireFlyOffsetFreeMPC_bmy18);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld18, FireFlyOffsetFreeMPC_bmy18, FireFlyOffsetFreeMPC_yy18);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld18, FireFlyOffsetFreeMPC_yy18, FireFlyOffsetFreeMPC_dvaff18);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd18, FireFlyOffsetFreeMPC_dvaff18, FireFlyOffsetFreeMPC_yy17, FireFlyOffsetFreeMPC_bmy17);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld17, FireFlyOffsetFreeMPC_bmy17, FireFlyOffsetFreeMPC_dvaff17);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd17, FireFlyOffsetFreeMPC_dvaff17, FireFlyOffsetFreeMPC_yy16, FireFlyOffsetFreeMPC_bmy16);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld16, FireFlyOffsetFreeMPC_bmy16, FireFlyOffsetFreeMPC_dvaff16);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd16, FireFlyOffsetFreeMPC_dvaff16, FireFlyOffsetFreeMPC_yy15, FireFlyOffsetFreeMPC_bmy15);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld15, FireFlyOffsetFreeMPC_bmy15, FireFlyOffsetFreeMPC_dvaff15);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd15, FireFlyOffsetFreeMPC_dvaff15, FireFlyOffsetFreeMPC_yy14, FireFlyOffsetFreeMPC_bmy14);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld14, FireFlyOffsetFreeMPC_bmy14, FireFlyOffsetFreeMPC_dvaff14);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd14, FireFlyOffsetFreeMPC_dvaff14, FireFlyOffsetFreeMPC_yy13, FireFlyOffsetFreeMPC_bmy13);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld13, FireFlyOffsetFreeMPC_bmy13, FireFlyOffsetFreeMPC_dvaff13);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd13, FireFlyOffsetFreeMPC_dvaff13, FireFlyOffsetFreeMPC_yy12, FireFlyOffsetFreeMPC_bmy12);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld12, FireFlyOffsetFreeMPC_bmy12, FireFlyOffsetFreeMPC_dvaff12);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd12, FireFlyOffsetFreeMPC_dvaff12, FireFlyOffsetFreeMPC_yy11, FireFlyOffsetFreeMPC_bmy11);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld11, FireFlyOffsetFreeMPC_bmy11, FireFlyOffsetFreeMPC_dvaff11);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd11, FireFlyOffsetFreeMPC_dvaff11, FireFlyOffsetFreeMPC_yy10, FireFlyOffsetFreeMPC_bmy10);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld10, FireFlyOffsetFreeMPC_bmy10, FireFlyOffsetFreeMPC_dvaff10);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd10, FireFlyOffsetFreeMPC_dvaff10, FireFlyOffsetFreeMPC_yy09, FireFlyOffsetFreeMPC_bmy09);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld09, FireFlyOffsetFreeMPC_bmy09, FireFlyOffsetFreeMPC_dvaff09);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd09, FireFlyOffsetFreeMPC_dvaff09, FireFlyOffsetFreeMPC_yy08, FireFlyOffsetFreeMPC_bmy08);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld08, FireFlyOffsetFreeMPC_bmy08, FireFlyOffsetFreeMPC_dvaff08);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd08, FireFlyOffsetFreeMPC_dvaff08, FireFlyOffsetFreeMPC_yy07, FireFlyOffsetFreeMPC_bmy07);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld07, FireFlyOffsetFreeMPC_bmy07, FireFlyOffsetFreeMPC_dvaff07);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd07, FireFlyOffsetFreeMPC_dvaff07, FireFlyOffsetFreeMPC_yy06, FireFlyOffsetFreeMPC_bmy06);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld06, FireFlyOffsetFreeMPC_bmy06, FireFlyOffsetFreeMPC_dvaff06);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd06, FireFlyOffsetFreeMPC_dvaff06, FireFlyOffsetFreeMPC_yy05, FireFlyOffsetFreeMPC_bmy05);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld05, FireFlyOffsetFreeMPC_bmy05, FireFlyOffsetFreeMPC_dvaff05);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd05, FireFlyOffsetFreeMPC_dvaff05, FireFlyOffsetFreeMPC_yy04, FireFlyOffsetFreeMPC_bmy04);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld04, FireFlyOffsetFreeMPC_bmy04, FireFlyOffsetFreeMPC_dvaff04);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd04, FireFlyOffsetFreeMPC_dvaff04, FireFlyOffsetFreeMPC_yy03, FireFlyOffsetFreeMPC_bmy03);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld03, FireFlyOffsetFreeMPC_bmy03, FireFlyOffsetFreeMPC_dvaff03);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd03, FireFlyOffsetFreeMPC_dvaff03, FireFlyOffsetFreeMPC_yy02, FireFlyOffsetFreeMPC_bmy02);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld02, FireFlyOffsetFreeMPC_bmy02, FireFlyOffsetFreeMPC_dvaff02);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd02, FireFlyOffsetFreeMPC_dvaff02, FireFlyOffsetFreeMPC_yy01, FireFlyOffsetFreeMPC_bmy01);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld01, FireFlyOffsetFreeMPC_bmy01, FireFlyOffsetFreeMPC_dvaff01);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_28(FireFlyOffsetFreeMPC_Lsd01, FireFlyOffsetFreeMPC_dvaff01, FireFlyOffsetFreeMPC_yy00, FireFlyOffsetFreeMPC_bmy00);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_28(FireFlyOffsetFreeMPC_Ld00, FireFlyOffsetFreeMPC_bmy00, FireFlyOffsetFreeMPC_dvaff00);
FireFlyOffsetFreeMPC_LA_DENSE_MTVM_28_17(FireFlyOffsetFreeMPC_C00, FireFlyOffsetFreeMPC_dvaff00, FireFlyOffsetFreeMPC_grad_eq00);
FireFlyOffsetFreeMPC_LA_DENSE_MTVM2_14_17_28(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff01, FireFlyOffsetFreeMPC_D01, FireFlyOffsetFreeMPC_dvaff00, FireFlyOffsetFreeMPC_grad_eq01);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff02, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff01, FireFlyOffsetFreeMPC_grad_eq02);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff03, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff02, FireFlyOffsetFreeMPC_grad_eq03);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff04, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff03, FireFlyOffsetFreeMPC_grad_eq04);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff05, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff04, FireFlyOffsetFreeMPC_grad_eq05);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff06, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff05, FireFlyOffsetFreeMPC_grad_eq06);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff07, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff06, FireFlyOffsetFreeMPC_grad_eq07);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff08, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff07, FireFlyOffsetFreeMPC_grad_eq08);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff09, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff08, FireFlyOffsetFreeMPC_grad_eq09);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff10, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff09, FireFlyOffsetFreeMPC_grad_eq10);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff11, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff10, FireFlyOffsetFreeMPC_grad_eq11);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff12, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff11, FireFlyOffsetFreeMPC_grad_eq12);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff13, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff12, FireFlyOffsetFreeMPC_grad_eq13);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff14, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff13, FireFlyOffsetFreeMPC_grad_eq14);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff15, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff14, FireFlyOffsetFreeMPC_grad_eq15);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff16, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff15, FireFlyOffsetFreeMPC_grad_eq16);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff17, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff16, FireFlyOffsetFreeMPC_grad_eq17);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvaff18, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvaff17, FireFlyOffsetFreeMPC_grad_eq18);
FireFlyOffsetFreeMPC_LA_DIAGZERO_MTVM_14_14(FireFlyOffsetFreeMPC_D19, FireFlyOffsetFreeMPC_dvaff18, FireFlyOffsetFreeMPC_grad_eq19);
FireFlyOffsetFreeMPC_LA_VSUB2_337(FireFlyOffsetFreeMPC_rd, FireFlyOffsetFreeMPC_grad_eq, FireFlyOffsetFreeMPC_rd);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi00, FireFlyOffsetFreeMPC_rd00, FireFlyOffsetFreeMPC_dzaff00);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi01, FireFlyOffsetFreeMPC_rd01, FireFlyOffsetFreeMPC_dzaff01);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi02, FireFlyOffsetFreeMPC_rd02, FireFlyOffsetFreeMPC_dzaff02);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi03, FireFlyOffsetFreeMPC_rd03, FireFlyOffsetFreeMPC_dzaff03);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi04, FireFlyOffsetFreeMPC_rd04, FireFlyOffsetFreeMPC_dzaff04);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi05, FireFlyOffsetFreeMPC_rd05, FireFlyOffsetFreeMPC_dzaff05);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi06, FireFlyOffsetFreeMPC_rd06, FireFlyOffsetFreeMPC_dzaff06);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi07, FireFlyOffsetFreeMPC_rd07, FireFlyOffsetFreeMPC_dzaff07);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi08, FireFlyOffsetFreeMPC_rd08, FireFlyOffsetFreeMPC_dzaff08);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi09, FireFlyOffsetFreeMPC_rd09, FireFlyOffsetFreeMPC_dzaff09);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi10, FireFlyOffsetFreeMPC_rd10, FireFlyOffsetFreeMPC_dzaff10);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi11, FireFlyOffsetFreeMPC_rd11, FireFlyOffsetFreeMPC_dzaff11);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi12, FireFlyOffsetFreeMPC_rd12, FireFlyOffsetFreeMPC_dzaff12);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi13, FireFlyOffsetFreeMPC_rd13, FireFlyOffsetFreeMPC_dzaff13);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi14, FireFlyOffsetFreeMPC_rd14, FireFlyOffsetFreeMPC_dzaff14);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi15, FireFlyOffsetFreeMPC_rd15, FireFlyOffsetFreeMPC_dzaff15);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi16, FireFlyOffsetFreeMPC_rd16, FireFlyOffsetFreeMPC_dzaff16);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi17, FireFlyOffsetFreeMPC_rd17, FireFlyOffsetFreeMPC_dzaff17);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi18, FireFlyOffsetFreeMPC_rd18, FireFlyOffsetFreeMPC_dzaff18);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_14(FireFlyOffsetFreeMPC_Phi19, FireFlyOffsetFreeMPC_rd19, FireFlyOffsetFreeMPC_dzaff19);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff00, FireFlyOffsetFreeMPC_lbIdx00, FireFlyOffsetFreeMPC_rilb00, FireFlyOffsetFreeMPC_dslbaff00);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb00, FireFlyOffsetFreeMPC_dslbaff00, FireFlyOffsetFreeMPC_llb00, FireFlyOffsetFreeMPC_dllbaff00);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub00, FireFlyOffsetFreeMPC_dzaff00, FireFlyOffsetFreeMPC_ubIdx00, FireFlyOffsetFreeMPC_dsubaff00);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub00, FireFlyOffsetFreeMPC_dsubaff00, FireFlyOffsetFreeMPC_lub00, FireFlyOffsetFreeMPC_dlubaff00);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff01, FireFlyOffsetFreeMPC_lbIdx01, FireFlyOffsetFreeMPC_rilb01, FireFlyOffsetFreeMPC_dslbaff01);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb01, FireFlyOffsetFreeMPC_dslbaff01, FireFlyOffsetFreeMPC_llb01, FireFlyOffsetFreeMPC_dllbaff01);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub01, FireFlyOffsetFreeMPC_dzaff01, FireFlyOffsetFreeMPC_ubIdx01, FireFlyOffsetFreeMPC_dsubaff01);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub01, FireFlyOffsetFreeMPC_dsubaff01, FireFlyOffsetFreeMPC_lub01, FireFlyOffsetFreeMPC_dlubaff01);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff02, FireFlyOffsetFreeMPC_lbIdx02, FireFlyOffsetFreeMPC_rilb02, FireFlyOffsetFreeMPC_dslbaff02);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb02, FireFlyOffsetFreeMPC_dslbaff02, FireFlyOffsetFreeMPC_llb02, FireFlyOffsetFreeMPC_dllbaff02);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub02, FireFlyOffsetFreeMPC_dzaff02, FireFlyOffsetFreeMPC_ubIdx02, FireFlyOffsetFreeMPC_dsubaff02);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub02, FireFlyOffsetFreeMPC_dsubaff02, FireFlyOffsetFreeMPC_lub02, FireFlyOffsetFreeMPC_dlubaff02);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff03, FireFlyOffsetFreeMPC_lbIdx03, FireFlyOffsetFreeMPC_rilb03, FireFlyOffsetFreeMPC_dslbaff03);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb03, FireFlyOffsetFreeMPC_dslbaff03, FireFlyOffsetFreeMPC_llb03, FireFlyOffsetFreeMPC_dllbaff03);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub03, FireFlyOffsetFreeMPC_dzaff03, FireFlyOffsetFreeMPC_ubIdx03, FireFlyOffsetFreeMPC_dsubaff03);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub03, FireFlyOffsetFreeMPC_dsubaff03, FireFlyOffsetFreeMPC_lub03, FireFlyOffsetFreeMPC_dlubaff03);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff04, FireFlyOffsetFreeMPC_lbIdx04, FireFlyOffsetFreeMPC_rilb04, FireFlyOffsetFreeMPC_dslbaff04);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb04, FireFlyOffsetFreeMPC_dslbaff04, FireFlyOffsetFreeMPC_llb04, FireFlyOffsetFreeMPC_dllbaff04);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub04, FireFlyOffsetFreeMPC_dzaff04, FireFlyOffsetFreeMPC_ubIdx04, FireFlyOffsetFreeMPC_dsubaff04);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub04, FireFlyOffsetFreeMPC_dsubaff04, FireFlyOffsetFreeMPC_lub04, FireFlyOffsetFreeMPC_dlubaff04);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff05, FireFlyOffsetFreeMPC_lbIdx05, FireFlyOffsetFreeMPC_rilb05, FireFlyOffsetFreeMPC_dslbaff05);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb05, FireFlyOffsetFreeMPC_dslbaff05, FireFlyOffsetFreeMPC_llb05, FireFlyOffsetFreeMPC_dllbaff05);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub05, FireFlyOffsetFreeMPC_dzaff05, FireFlyOffsetFreeMPC_ubIdx05, FireFlyOffsetFreeMPC_dsubaff05);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub05, FireFlyOffsetFreeMPC_dsubaff05, FireFlyOffsetFreeMPC_lub05, FireFlyOffsetFreeMPC_dlubaff05);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff06, FireFlyOffsetFreeMPC_lbIdx06, FireFlyOffsetFreeMPC_rilb06, FireFlyOffsetFreeMPC_dslbaff06);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb06, FireFlyOffsetFreeMPC_dslbaff06, FireFlyOffsetFreeMPC_llb06, FireFlyOffsetFreeMPC_dllbaff06);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub06, FireFlyOffsetFreeMPC_dzaff06, FireFlyOffsetFreeMPC_ubIdx06, FireFlyOffsetFreeMPC_dsubaff06);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub06, FireFlyOffsetFreeMPC_dsubaff06, FireFlyOffsetFreeMPC_lub06, FireFlyOffsetFreeMPC_dlubaff06);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff07, FireFlyOffsetFreeMPC_lbIdx07, FireFlyOffsetFreeMPC_rilb07, FireFlyOffsetFreeMPC_dslbaff07);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb07, FireFlyOffsetFreeMPC_dslbaff07, FireFlyOffsetFreeMPC_llb07, FireFlyOffsetFreeMPC_dllbaff07);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub07, FireFlyOffsetFreeMPC_dzaff07, FireFlyOffsetFreeMPC_ubIdx07, FireFlyOffsetFreeMPC_dsubaff07);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub07, FireFlyOffsetFreeMPC_dsubaff07, FireFlyOffsetFreeMPC_lub07, FireFlyOffsetFreeMPC_dlubaff07);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff08, FireFlyOffsetFreeMPC_lbIdx08, FireFlyOffsetFreeMPC_rilb08, FireFlyOffsetFreeMPC_dslbaff08);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb08, FireFlyOffsetFreeMPC_dslbaff08, FireFlyOffsetFreeMPC_llb08, FireFlyOffsetFreeMPC_dllbaff08);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub08, FireFlyOffsetFreeMPC_dzaff08, FireFlyOffsetFreeMPC_ubIdx08, FireFlyOffsetFreeMPC_dsubaff08);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub08, FireFlyOffsetFreeMPC_dsubaff08, FireFlyOffsetFreeMPC_lub08, FireFlyOffsetFreeMPC_dlubaff08);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff09, FireFlyOffsetFreeMPC_lbIdx09, FireFlyOffsetFreeMPC_rilb09, FireFlyOffsetFreeMPC_dslbaff09);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb09, FireFlyOffsetFreeMPC_dslbaff09, FireFlyOffsetFreeMPC_llb09, FireFlyOffsetFreeMPC_dllbaff09);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub09, FireFlyOffsetFreeMPC_dzaff09, FireFlyOffsetFreeMPC_ubIdx09, FireFlyOffsetFreeMPC_dsubaff09);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub09, FireFlyOffsetFreeMPC_dsubaff09, FireFlyOffsetFreeMPC_lub09, FireFlyOffsetFreeMPC_dlubaff09);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff10, FireFlyOffsetFreeMPC_lbIdx10, FireFlyOffsetFreeMPC_rilb10, FireFlyOffsetFreeMPC_dslbaff10);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb10, FireFlyOffsetFreeMPC_dslbaff10, FireFlyOffsetFreeMPC_llb10, FireFlyOffsetFreeMPC_dllbaff10);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub10, FireFlyOffsetFreeMPC_dzaff10, FireFlyOffsetFreeMPC_ubIdx10, FireFlyOffsetFreeMPC_dsubaff10);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub10, FireFlyOffsetFreeMPC_dsubaff10, FireFlyOffsetFreeMPC_lub10, FireFlyOffsetFreeMPC_dlubaff10);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff11, FireFlyOffsetFreeMPC_lbIdx11, FireFlyOffsetFreeMPC_rilb11, FireFlyOffsetFreeMPC_dslbaff11);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb11, FireFlyOffsetFreeMPC_dslbaff11, FireFlyOffsetFreeMPC_llb11, FireFlyOffsetFreeMPC_dllbaff11);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub11, FireFlyOffsetFreeMPC_dzaff11, FireFlyOffsetFreeMPC_ubIdx11, FireFlyOffsetFreeMPC_dsubaff11);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub11, FireFlyOffsetFreeMPC_dsubaff11, FireFlyOffsetFreeMPC_lub11, FireFlyOffsetFreeMPC_dlubaff11);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff12, FireFlyOffsetFreeMPC_lbIdx12, FireFlyOffsetFreeMPC_rilb12, FireFlyOffsetFreeMPC_dslbaff12);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb12, FireFlyOffsetFreeMPC_dslbaff12, FireFlyOffsetFreeMPC_llb12, FireFlyOffsetFreeMPC_dllbaff12);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub12, FireFlyOffsetFreeMPC_dzaff12, FireFlyOffsetFreeMPC_ubIdx12, FireFlyOffsetFreeMPC_dsubaff12);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub12, FireFlyOffsetFreeMPC_dsubaff12, FireFlyOffsetFreeMPC_lub12, FireFlyOffsetFreeMPC_dlubaff12);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff13, FireFlyOffsetFreeMPC_lbIdx13, FireFlyOffsetFreeMPC_rilb13, FireFlyOffsetFreeMPC_dslbaff13);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb13, FireFlyOffsetFreeMPC_dslbaff13, FireFlyOffsetFreeMPC_llb13, FireFlyOffsetFreeMPC_dllbaff13);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub13, FireFlyOffsetFreeMPC_dzaff13, FireFlyOffsetFreeMPC_ubIdx13, FireFlyOffsetFreeMPC_dsubaff13);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub13, FireFlyOffsetFreeMPC_dsubaff13, FireFlyOffsetFreeMPC_lub13, FireFlyOffsetFreeMPC_dlubaff13);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff14, FireFlyOffsetFreeMPC_lbIdx14, FireFlyOffsetFreeMPC_rilb14, FireFlyOffsetFreeMPC_dslbaff14);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb14, FireFlyOffsetFreeMPC_dslbaff14, FireFlyOffsetFreeMPC_llb14, FireFlyOffsetFreeMPC_dllbaff14);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub14, FireFlyOffsetFreeMPC_dzaff14, FireFlyOffsetFreeMPC_ubIdx14, FireFlyOffsetFreeMPC_dsubaff14);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub14, FireFlyOffsetFreeMPC_dsubaff14, FireFlyOffsetFreeMPC_lub14, FireFlyOffsetFreeMPC_dlubaff14);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff15, FireFlyOffsetFreeMPC_lbIdx15, FireFlyOffsetFreeMPC_rilb15, FireFlyOffsetFreeMPC_dslbaff15);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb15, FireFlyOffsetFreeMPC_dslbaff15, FireFlyOffsetFreeMPC_llb15, FireFlyOffsetFreeMPC_dllbaff15);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub15, FireFlyOffsetFreeMPC_dzaff15, FireFlyOffsetFreeMPC_ubIdx15, FireFlyOffsetFreeMPC_dsubaff15);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub15, FireFlyOffsetFreeMPC_dsubaff15, FireFlyOffsetFreeMPC_lub15, FireFlyOffsetFreeMPC_dlubaff15);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff16, FireFlyOffsetFreeMPC_lbIdx16, FireFlyOffsetFreeMPC_rilb16, FireFlyOffsetFreeMPC_dslbaff16);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb16, FireFlyOffsetFreeMPC_dslbaff16, FireFlyOffsetFreeMPC_llb16, FireFlyOffsetFreeMPC_dllbaff16);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub16, FireFlyOffsetFreeMPC_dzaff16, FireFlyOffsetFreeMPC_ubIdx16, FireFlyOffsetFreeMPC_dsubaff16);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub16, FireFlyOffsetFreeMPC_dsubaff16, FireFlyOffsetFreeMPC_lub16, FireFlyOffsetFreeMPC_dlubaff16);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff17, FireFlyOffsetFreeMPC_lbIdx17, FireFlyOffsetFreeMPC_rilb17, FireFlyOffsetFreeMPC_dslbaff17);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb17, FireFlyOffsetFreeMPC_dslbaff17, FireFlyOffsetFreeMPC_llb17, FireFlyOffsetFreeMPC_dllbaff17);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub17, FireFlyOffsetFreeMPC_dzaff17, FireFlyOffsetFreeMPC_ubIdx17, FireFlyOffsetFreeMPC_dsubaff17);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub17, FireFlyOffsetFreeMPC_dsubaff17, FireFlyOffsetFreeMPC_lub17, FireFlyOffsetFreeMPC_dlubaff17);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_6(FireFlyOffsetFreeMPC_dzaff18, FireFlyOffsetFreeMPC_lbIdx18, FireFlyOffsetFreeMPC_rilb18, FireFlyOffsetFreeMPC_dslbaff18);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_llbbyslb18, FireFlyOffsetFreeMPC_dslbaff18, FireFlyOffsetFreeMPC_llb18, FireFlyOffsetFreeMPC_dllbaff18);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_6(FireFlyOffsetFreeMPC_riub18, FireFlyOffsetFreeMPC_dzaff18, FireFlyOffsetFreeMPC_ubIdx18, FireFlyOffsetFreeMPC_dsubaff18);
FireFlyOffsetFreeMPC_LA_VSUB3_6(FireFlyOffsetFreeMPC_lubbysub18, FireFlyOffsetFreeMPC_dsubaff18, FireFlyOffsetFreeMPC_lub18, FireFlyOffsetFreeMPC_dlubaff18);
FireFlyOffsetFreeMPC_LA_VSUB_INDEXED_3(FireFlyOffsetFreeMPC_dzaff19, FireFlyOffsetFreeMPC_lbIdx19, FireFlyOffsetFreeMPC_rilb19, FireFlyOffsetFreeMPC_dslbaff19);
FireFlyOffsetFreeMPC_LA_VSUB3_3(FireFlyOffsetFreeMPC_llbbyslb19, FireFlyOffsetFreeMPC_dslbaff19, FireFlyOffsetFreeMPC_llb19, FireFlyOffsetFreeMPC_dllbaff19);
FireFlyOffsetFreeMPC_LA_VSUB2_INDEXED_3(FireFlyOffsetFreeMPC_riub19, FireFlyOffsetFreeMPC_dzaff19, FireFlyOffsetFreeMPC_ubIdx19, FireFlyOffsetFreeMPC_dsubaff19);
FireFlyOffsetFreeMPC_LA_VSUB3_3(FireFlyOffsetFreeMPC_lubbysub19, FireFlyOffsetFreeMPC_dsubaff19, FireFlyOffsetFreeMPC_lub19, FireFlyOffsetFreeMPC_dlubaff19);
info->lsit_aff = FireFlyOffsetFreeMPC_LINESEARCH_BACKTRACKING_AFFINE(FireFlyOffsetFreeMPC_l, FireFlyOffsetFreeMPC_s, FireFlyOffsetFreeMPC_dl_aff, FireFlyOffsetFreeMPC_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == FireFlyOffsetFreeMPC_NOPROGRESS ){
exitcode = FireFlyOffsetFreeMPC_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
FireFlyOffsetFreeMPC_LA_VSUB5_234(FireFlyOffsetFreeMPC_ds_aff, FireFlyOffsetFreeMPC_dl_aff, info->mu, info->sigma, FireFlyOffsetFreeMPC_ccrhs);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub00, FireFlyOffsetFreeMPC_sub00, FireFlyOffsetFreeMPC_ubIdx00, FireFlyOffsetFreeMPC_ccrhsl00, FireFlyOffsetFreeMPC_slb00, FireFlyOffsetFreeMPC_lbIdx00, FireFlyOffsetFreeMPC_rd00);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub01, FireFlyOffsetFreeMPC_sub01, FireFlyOffsetFreeMPC_ubIdx01, FireFlyOffsetFreeMPC_ccrhsl01, FireFlyOffsetFreeMPC_slb01, FireFlyOffsetFreeMPC_lbIdx01, FireFlyOffsetFreeMPC_rd01);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi00, FireFlyOffsetFreeMPC_rd00, FireFlyOffsetFreeMPC_Lbyrd00);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi01, FireFlyOffsetFreeMPC_rd01, FireFlyOffsetFreeMPC_Lbyrd01);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_28_17_17(FireFlyOffsetFreeMPC_V00, FireFlyOffsetFreeMPC_Lbyrd00, FireFlyOffsetFreeMPC_W01, FireFlyOffsetFreeMPC_Lbyrd01, FireFlyOffsetFreeMPC_beta00);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_28(FireFlyOffsetFreeMPC_Ld00, FireFlyOffsetFreeMPC_beta00, FireFlyOffsetFreeMPC_yy00);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub02, FireFlyOffsetFreeMPC_sub02, FireFlyOffsetFreeMPC_ubIdx02, FireFlyOffsetFreeMPC_ccrhsl02, FireFlyOffsetFreeMPC_slb02, FireFlyOffsetFreeMPC_lbIdx02, FireFlyOffsetFreeMPC_rd02);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi02, FireFlyOffsetFreeMPC_rd02, FireFlyOffsetFreeMPC_Lbyrd02);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V01, FireFlyOffsetFreeMPC_Lbyrd01, FireFlyOffsetFreeMPC_W02, FireFlyOffsetFreeMPC_Lbyrd02, FireFlyOffsetFreeMPC_beta01);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_28(FireFlyOffsetFreeMPC_Lsd01, FireFlyOffsetFreeMPC_yy00, FireFlyOffsetFreeMPC_beta01, FireFlyOffsetFreeMPC_bmy01);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld01, FireFlyOffsetFreeMPC_bmy01, FireFlyOffsetFreeMPC_yy01);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub03, FireFlyOffsetFreeMPC_sub03, FireFlyOffsetFreeMPC_ubIdx03, FireFlyOffsetFreeMPC_ccrhsl03, FireFlyOffsetFreeMPC_slb03, FireFlyOffsetFreeMPC_lbIdx03, FireFlyOffsetFreeMPC_rd03);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi03, FireFlyOffsetFreeMPC_rd03, FireFlyOffsetFreeMPC_Lbyrd03);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V02, FireFlyOffsetFreeMPC_Lbyrd02, FireFlyOffsetFreeMPC_W03, FireFlyOffsetFreeMPC_Lbyrd03, FireFlyOffsetFreeMPC_beta02);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd02, FireFlyOffsetFreeMPC_yy01, FireFlyOffsetFreeMPC_beta02, FireFlyOffsetFreeMPC_bmy02);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld02, FireFlyOffsetFreeMPC_bmy02, FireFlyOffsetFreeMPC_yy02);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub04, FireFlyOffsetFreeMPC_sub04, FireFlyOffsetFreeMPC_ubIdx04, FireFlyOffsetFreeMPC_ccrhsl04, FireFlyOffsetFreeMPC_slb04, FireFlyOffsetFreeMPC_lbIdx04, FireFlyOffsetFreeMPC_rd04);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi04, FireFlyOffsetFreeMPC_rd04, FireFlyOffsetFreeMPC_Lbyrd04);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V03, FireFlyOffsetFreeMPC_Lbyrd03, FireFlyOffsetFreeMPC_W04, FireFlyOffsetFreeMPC_Lbyrd04, FireFlyOffsetFreeMPC_beta03);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd03, FireFlyOffsetFreeMPC_yy02, FireFlyOffsetFreeMPC_beta03, FireFlyOffsetFreeMPC_bmy03);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld03, FireFlyOffsetFreeMPC_bmy03, FireFlyOffsetFreeMPC_yy03);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub05, FireFlyOffsetFreeMPC_sub05, FireFlyOffsetFreeMPC_ubIdx05, FireFlyOffsetFreeMPC_ccrhsl05, FireFlyOffsetFreeMPC_slb05, FireFlyOffsetFreeMPC_lbIdx05, FireFlyOffsetFreeMPC_rd05);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi05, FireFlyOffsetFreeMPC_rd05, FireFlyOffsetFreeMPC_Lbyrd05);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V04, FireFlyOffsetFreeMPC_Lbyrd04, FireFlyOffsetFreeMPC_W05, FireFlyOffsetFreeMPC_Lbyrd05, FireFlyOffsetFreeMPC_beta04);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd04, FireFlyOffsetFreeMPC_yy03, FireFlyOffsetFreeMPC_beta04, FireFlyOffsetFreeMPC_bmy04);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld04, FireFlyOffsetFreeMPC_bmy04, FireFlyOffsetFreeMPC_yy04);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub06, FireFlyOffsetFreeMPC_sub06, FireFlyOffsetFreeMPC_ubIdx06, FireFlyOffsetFreeMPC_ccrhsl06, FireFlyOffsetFreeMPC_slb06, FireFlyOffsetFreeMPC_lbIdx06, FireFlyOffsetFreeMPC_rd06);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi06, FireFlyOffsetFreeMPC_rd06, FireFlyOffsetFreeMPC_Lbyrd06);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V05, FireFlyOffsetFreeMPC_Lbyrd05, FireFlyOffsetFreeMPC_W06, FireFlyOffsetFreeMPC_Lbyrd06, FireFlyOffsetFreeMPC_beta05);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd05, FireFlyOffsetFreeMPC_yy04, FireFlyOffsetFreeMPC_beta05, FireFlyOffsetFreeMPC_bmy05);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld05, FireFlyOffsetFreeMPC_bmy05, FireFlyOffsetFreeMPC_yy05);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub07, FireFlyOffsetFreeMPC_sub07, FireFlyOffsetFreeMPC_ubIdx07, FireFlyOffsetFreeMPC_ccrhsl07, FireFlyOffsetFreeMPC_slb07, FireFlyOffsetFreeMPC_lbIdx07, FireFlyOffsetFreeMPC_rd07);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi07, FireFlyOffsetFreeMPC_rd07, FireFlyOffsetFreeMPC_Lbyrd07);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V06, FireFlyOffsetFreeMPC_Lbyrd06, FireFlyOffsetFreeMPC_W07, FireFlyOffsetFreeMPC_Lbyrd07, FireFlyOffsetFreeMPC_beta06);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd06, FireFlyOffsetFreeMPC_yy05, FireFlyOffsetFreeMPC_beta06, FireFlyOffsetFreeMPC_bmy06);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld06, FireFlyOffsetFreeMPC_bmy06, FireFlyOffsetFreeMPC_yy06);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub08, FireFlyOffsetFreeMPC_sub08, FireFlyOffsetFreeMPC_ubIdx08, FireFlyOffsetFreeMPC_ccrhsl08, FireFlyOffsetFreeMPC_slb08, FireFlyOffsetFreeMPC_lbIdx08, FireFlyOffsetFreeMPC_rd08);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi08, FireFlyOffsetFreeMPC_rd08, FireFlyOffsetFreeMPC_Lbyrd08);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V07, FireFlyOffsetFreeMPC_Lbyrd07, FireFlyOffsetFreeMPC_W08, FireFlyOffsetFreeMPC_Lbyrd08, FireFlyOffsetFreeMPC_beta07);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd07, FireFlyOffsetFreeMPC_yy06, FireFlyOffsetFreeMPC_beta07, FireFlyOffsetFreeMPC_bmy07);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld07, FireFlyOffsetFreeMPC_bmy07, FireFlyOffsetFreeMPC_yy07);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub09, FireFlyOffsetFreeMPC_sub09, FireFlyOffsetFreeMPC_ubIdx09, FireFlyOffsetFreeMPC_ccrhsl09, FireFlyOffsetFreeMPC_slb09, FireFlyOffsetFreeMPC_lbIdx09, FireFlyOffsetFreeMPC_rd09);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi09, FireFlyOffsetFreeMPC_rd09, FireFlyOffsetFreeMPC_Lbyrd09);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V08, FireFlyOffsetFreeMPC_Lbyrd08, FireFlyOffsetFreeMPC_W09, FireFlyOffsetFreeMPC_Lbyrd09, FireFlyOffsetFreeMPC_beta08);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd08, FireFlyOffsetFreeMPC_yy07, FireFlyOffsetFreeMPC_beta08, FireFlyOffsetFreeMPC_bmy08);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld08, FireFlyOffsetFreeMPC_bmy08, FireFlyOffsetFreeMPC_yy08);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub10, FireFlyOffsetFreeMPC_sub10, FireFlyOffsetFreeMPC_ubIdx10, FireFlyOffsetFreeMPC_ccrhsl10, FireFlyOffsetFreeMPC_slb10, FireFlyOffsetFreeMPC_lbIdx10, FireFlyOffsetFreeMPC_rd10);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi10, FireFlyOffsetFreeMPC_rd10, FireFlyOffsetFreeMPC_Lbyrd10);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V09, FireFlyOffsetFreeMPC_Lbyrd09, FireFlyOffsetFreeMPC_W10, FireFlyOffsetFreeMPC_Lbyrd10, FireFlyOffsetFreeMPC_beta09);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd09, FireFlyOffsetFreeMPC_yy08, FireFlyOffsetFreeMPC_beta09, FireFlyOffsetFreeMPC_bmy09);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld09, FireFlyOffsetFreeMPC_bmy09, FireFlyOffsetFreeMPC_yy09);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub11, FireFlyOffsetFreeMPC_sub11, FireFlyOffsetFreeMPC_ubIdx11, FireFlyOffsetFreeMPC_ccrhsl11, FireFlyOffsetFreeMPC_slb11, FireFlyOffsetFreeMPC_lbIdx11, FireFlyOffsetFreeMPC_rd11);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi11, FireFlyOffsetFreeMPC_rd11, FireFlyOffsetFreeMPC_Lbyrd11);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V10, FireFlyOffsetFreeMPC_Lbyrd10, FireFlyOffsetFreeMPC_W11, FireFlyOffsetFreeMPC_Lbyrd11, FireFlyOffsetFreeMPC_beta10);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd10, FireFlyOffsetFreeMPC_yy09, FireFlyOffsetFreeMPC_beta10, FireFlyOffsetFreeMPC_bmy10);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld10, FireFlyOffsetFreeMPC_bmy10, FireFlyOffsetFreeMPC_yy10);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub12, FireFlyOffsetFreeMPC_sub12, FireFlyOffsetFreeMPC_ubIdx12, FireFlyOffsetFreeMPC_ccrhsl12, FireFlyOffsetFreeMPC_slb12, FireFlyOffsetFreeMPC_lbIdx12, FireFlyOffsetFreeMPC_rd12);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi12, FireFlyOffsetFreeMPC_rd12, FireFlyOffsetFreeMPC_Lbyrd12);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V11, FireFlyOffsetFreeMPC_Lbyrd11, FireFlyOffsetFreeMPC_W12, FireFlyOffsetFreeMPC_Lbyrd12, FireFlyOffsetFreeMPC_beta11);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd11, FireFlyOffsetFreeMPC_yy10, FireFlyOffsetFreeMPC_beta11, FireFlyOffsetFreeMPC_bmy11);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld11, FireFlyOffsetFreeMPC_bmy11, FireFlyOffsetFreeMPC_yy11);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub13, FireFlyOffsetFreeMPC_sub13, FireFlyOffsetFreeMPC_ubIdx13, FireFlyOffsetFreeMPC_ccrhsl13, FireFlyOffsetFreeMPC_slb13, FireFlyOffsetFreeMPC_lbIdx13, FireFlyOffsetFreeMPC_rd13);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi13, FireFlyOffsetFreeMPC_rd13, FireFlyOffsetFreeMPC_Lbyrd13);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V12, FireFlyOffsetFreeMPC_Lbyrd12, FireFlyOffsetFreeMPC_W13, FireFlyOffsetFreeMPC_Lbyrd13, FireFlyOffsetFreeMPC_beta12);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd12, FireFlyOffsetFreeMPC_yy11, FireFlyOffsetFreeMPC_beta12, FireFlyOffsetFreeMPC_bmy12);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld12, FireFlyOffsetFreeMPC_bmy12, FireFlyOffsetFreeMPC_yy12);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub14, FireFlyOffsetFreeMPC_sub14, FireFlyOffsetFreeMPC_ubIdx14, FireFlyOffsetFreeMPC_ccrhsl14, FireFlyOffsetFreeMPC_slb14, FireFlyOffsetFreeMPC_lbIdx14, FireFlyOffsetFreeMPC_rd14);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi14, FireFlyOffsetFreeMPC_rd14, FireFlyOffsetFreeMPC_Lbyrd14);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V13, FireFlyOffsetFreeMPC_Lbyrd13, FireFlyOffsetFreeMPC_W14, FireFlyOffsetFreeMPC_Lbyrd14, FireFlyOffsetFreeMPC_beta13);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd13, FireFlyOffsetFreeMPC_yy12, FireFlyOffsetFreeMPC_beta13, FireFlyOffsetFreeMPC_bmy13);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld13, FireFlyOffsetFreeMPC_bmy13, FireFlyOffsetFreeMPC_yy13);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub15, FireFlyOffsetFreeMPC_sub15, FireFlyOffsetFreeMPC_ubIdx15, FireFlyOffsetFreeMPC_ccrhsl15, FireFlyOffsetFreeMPC_slb15, FireFlyOffsetFreeMPC_lbIdx15, FireFlyOffsetFreeMPC_rd15);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi15, FireFlyOffsetFreeMPC_rd15, FireFlyOffsetFreeMPC_Lbyrd15);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V14, FireFlyOffsetFreeMPC_Lbyrd14, FireFlyOffsetFreeMPC_W15, FireFlyOffsetFreeMPC_Lbyrd15, FireFlyOffsetFreeMPC_beta14);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd14, FireFlyOffsetFreeMPC_yy13, FireFlyOffsetFreeMPC_beta14, FireFlyOffsetFreeMPC_bmy14);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld14, FireFlyOffsetFreeMPC_bmy14, FireFlyOffsetFreeMPC_yy14);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub16, FireFlyOffsetFreeMPC_sub16, FireFlyOffsetFreeMPC_ubIdx16, FireFlyOffsetFreeMPC_ccrhsl16, FireFlyOffsetFreeMPC_slb16, FireFlyOffsetFreeMPC_lbIdx16, FireFlyOffsetFreeMPC_rd16);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi16, FireFlyOffsetFreeMPC_rd16, FireFlyOffsetFreeMPC_Lbyrd16);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V15, FireFlyOffsetFreeMPC_Lbyrd15, FireFlyOffsetFreeMPC_W16, FireFlyOffsetFreeMPC_Lbyrd16, FireFlyOffsetFreeMPC_beta15);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd15, FireFlyOffsetFreeMPC_yy14, FireFlyOffsetFreeMPC_beta15, FireFlyOffsetFreeMPC_bmy15);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld15, FireFlyOffsetFreeMPC_bmy15, FireFlyOffsetFreeMPC_yy15);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub17, FireFlyOffsetFreeMPC_sub17, FireFlyOffsetFreeMPC_ubIdx17, FireFlyOffsetFreeMPC_ccrhsl17, FireFlyOffsetFreeMPC_slb17, FireFlyOffsetFreeMPC_lbIdx17, FireFlyOffsetFreeMPC_rd17);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi17, FireFlyOffsetFreeMPC_rd17, FireFlyOffsetFreeMPC_Lbyrd17);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V16, FireFlyOffsetFreeMPC_Lbyrd16, FireFlyOffsetFreeMPC_W17, FireFlyOffsetFreeMPC_Lbyrd17, FireFlyOffsetFreeMPC_beta16);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd16, FireFlyOffsetFreeMPC_yy15, FireFlyOffsetFreeMPC_beta16, FireFlyOffsetFreeMPC_bmy16);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld16, FireFlyOffsetFreeMPC_bmy16, FireFlyOffsetFreeMPC_yy16);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_17_6_6(FireFlyOffsetFreeMPC_ccrhsub18, FireFlyOffsetFreeMPC_sub18, FireFlyOffsetFreeMPC_ubIdx18, FireFlyOffsetFreeMPC_ccrhsl18, FireFlyOffsetFreeMPC_slb18, FireFlyOffsetFreeMPC_lbIdx18, FireFlyOffsetFreeMPC_rd18);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_17(FireFlyOffsetFreeMPC_Phi18, FireFlyOffsetFreeMPC_rd18, FireFlyOffsetFreeMPC_Lbyrd18);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_17(FireFlyOffsetFreeMPC_V17, FireFlyOffsetFreeMPC_Lbyrd17, FireFlyOffsetFreeMPC_W18, FireFlyOffsetFreeMPC_Lbyrd18, FireFlyOffsetFreeMPC_beta17);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd17, FireFlyOffsetFreeMPC_yy16, FireFlyOffsetFreeMPC_beta17, FireFlyOffsetFreeMPC_bmy17);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld17, FireFlyOffsetFreeMPC_bmy17, FireFlyOffsetFreeMPC_yy17);
FireFlyOffsetFreeMPC_LA_VSUB6_INDEXED_14_3_3(FireFlyOffsetFreeMPC_ccrhsub19, FireFlyOffsetFreeMPC_sub19, FireFlyOffsetFreeMPC_ubIdx19, FireFlyOffsetFreeMPC_ccrhsl19, FireFlyOffsetFreeMPC_slb19, FireFlyOffsetFreeMPC_lbIdx19, FireFlyOffsetFreeMPC_rd19);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Phi19, FireFlyOffsetFreeMPC_rd19, FireFlyOffsetFreeMPC_Lbyrd19);
FireFlyOffsetFreeMPC_LA_DENSE_2MVMADD_14_17_14(FireFlyOffsetFreeMPC_V18, FireFlyOffsetFreeMPC_Lbyrd18, FireFlyOffsetFreeMPC_W19, FireFlyOffsetFreeMPC_Lbyrd19, FireFlyOffsetFreeMPC_beta18);
FireFlyOffsetFreeMPC_LA_DENSE_MVMSUB1_14_14(FireFlyOffsetFreeMPC_Lsd18, FireFlyOffsetFreeMPC_yy17, FireFlyOffsetFreeMPC_beta18, FireFlyOffsetFreeMPC_bmy18);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDSUB_14(FireFlyOffsetFreeMPC_Ld18, FireFlyOffsetFreeMPC_bmy18, FireFlyOffsetFreeMPC_yy18);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld18, FireFlyOffsetFreeMPC_yy18, FireFlyOffsetFreeMPC_dvcc18);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd18, FireFlyOffsetFreeMPC_dvcc18, FireFlyOffsetFreeMPC_yy17, FireFlyOffsetFreeMPC_bmy17);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld17, FireFlyOffsetFreeMPC_bmy17, FireFlyOffsetFreeMPC_dvcc17);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd17, FireFlyOffsetFreeMPC_dvcc17, FireFlyOffsetFreeMPC_yy16, FireFlyOffsetFreeMPC_bmy16);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld16, FireFlyOffsetFreeMPC_bmy16, FireFlyOffsetFreeMPC_dvcc16);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd16, FireFlyOffsetFreeMPC_dvcc16, FireFlyOffsetFreeMPC_yy15, FireFlyOffsetFreeMPC_bmy15);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld15, FireFlyOffsetFreeMPC_bmy15, FireFlyOffsetFreeMPC_dvcc15);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd15, FireFlyOffsetFreeMPC_dvcc15, FireFlyOffsetFreeMPC_yy14, FireFlyOffsetFreeMPC_bmy14);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld14, FireFlyOffsetFreeMPC_bmy14, FireFlyOffsetFreeMPC_dvcc14);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd14, FireFlyOffsetFreeMPC_dvcc14, FireFlyOffsetFreeMPC_yy13, FireFlyOffsetFreeMPC_bmy13);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld13, FireFlyOffsetFreeMPC_bmy13, FireFlyOffsetFreeMPC_dvcc13);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd13, FireFlyOffsetFreeMPC_dvcc13, FireFlyOffsetFreeMPC_yy12, FireFlyOffsetFreeMPC_bmy12);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld12, FireFlyOffsetFreeMPC_bmy12, FireFlyOffsetFreeMPC_dvcc12);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd12, FireFlyOffsetFreeMPC_dvcc12, FireFlyOffsetFreeMPC_yy11, FireFlyOffsetFreeMPC_bmy11);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld11, FireFlyOffsetFreeMPC_bmy11, FireFlyOffsetFreeMPC_dvcc11);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd11, FireFlyOffsetFreeMPC_dvcc11, FireFlyOffsetFreeMPC_yy10, FireFlyOffsetFreeMPC_bmy10);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld10, FireFlyOffsetFreeMPC_bmy10, FireFlyOffsetFreeMPC_dvcc10);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd10, FireFlyOffsetFreeMPC_dvcc10, FireFlyOffsetFreeMPC_yy09, FireFlyOffsetFreeMPC_bmy09);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld09, FireFlyOffsetFreeMPC_bmy09, FireFlyOffsetFreeMPC_dvcc09);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd09, FireFlyOffsetFreeMPC_dvcc09, FireFlyOffsetFreeMPC_yy08, FireFlyOffsetFreeMPC_bmy08);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld08, FireFlyOffsetFreeMPC_bmy08, FireFlyOffsetFreeMPC_dvcc08);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd08, FireFlyOffsetFreeMPC_dvcc08, FireFlyOffsetFreeMPC_yy07, FireFlyOffsetFreeMPC_bmy07);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld07, FireFlyOffsetFreeMPC_bmy07, FireFlyOffsetFreeMPC_dvcc07);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd07, FireFlyOffsetFreeMPC_dvcc07, FireFlyOffsetFreeMPC_yy06, FireFlyOffsetFreeMPC_bmy06);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld06, FireFlyOffsetFreeMPC_bmy06, FireFlyOffsetFreeMPC_dvcc06);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd06, FireFlyOffsetFreeMPC_dvcc06, FireFlyOffsetFreeMPC_yy05, FireFlyOffsetFreeMPC_bmy05);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld05, FireFlyOffsetFreeMPC_bmy05, FireFlyOffsetFreeMPC_dvcc05);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd05, FireFlyOffsetFreeMPC_dvcc05, FireFlyOffsetFreeMPC_yy04, FireFlyOffsetFreeMPC_bmy04);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld04, FireFlyOffsetFreeMPC_bmy04, FireFlyOffsetFreeMPC_dvcc04);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd04, FireFlyOffsetFreeMPC_dvcc04, FireFlyOffsetFreeMPC_yy03, FireFlyOffsetFreeMPC_bmy03);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld03, FireFlyOffsetFreeMPC_bmy03, FireFlyOffsetFreeMPC_dvcc03);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd03, FireFlyOffsetFreeMPC_dvcc03, FireFlyOffsetFreeMPC_yy02, FireFlyOffsetFreeMPC_bmy02);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld02, FireFlyOffsetFreeMPC_bmy02, FireFlyOffsetFreeMPC_dvcc02);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_14(FireFlyOffsetFreeMPC_Lsd02, FireFlyOffsetFreeMPC_dvcc02, FireFlyOffsetFreeMPC_yy01, FireFlyOffsetFreeMPC_bmy01);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_14(FireFlyOffsetFreeMPC_Ld01, FireFlyOffsetFreeMPC_bmy01, FireFlyOffsetFreeMPC_dvcc01);
FireFlyOffsetFreeMPC_LA_DENSE_MTVMSUB_14_28(FireFlyOffsetFreeMPC_Lsd01, FireFlyOffsetFreeMPC_dvcc01, FireFlyOffsetFreeMPC_yy00, FireFlyOffsetFreeMPC_bmy00);
FireFlyOffsetFreeMPC_LA_DENSE_BACKWARDSUB_28(FireFlyOffsetFreeMPC_Ld00, FireFlyOffsetFreeMPC_bmy00, FireFlyOffsetFreeMPC_dvcc00);
FireFlyOffsetFreeMPC_LA_DENSE_MTVM_28_17(FireFlyOffsetFreeMPC_C00, FireFlyOffsetFreeMPC_dvcc00, FireFlyOffsetFreeMPC_grad_eq00);
FireFlyOffsetFreeMPC_LA_DENSE_MTVM2_14_17_28(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc01, FireFlyOffsetFreeMPC_D01, FireFlyOffsetFreeMPC_dvcc00, FireFlyOffsetFreeMPC_grad_eq01);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc02, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc01, FireFlyOffsetFreeMPC_grad_eq02);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc03, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc02, FireFlyOffsetFreeMPC_grad_eq03);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc04, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc03, FireFlyOffsetFreeMPC_grad_eq04);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc05, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc04, FireFlyOffsetFreeMPC_grad_eq05);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc06, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc05, FireFlyOffsetFreeMPC_grad_eq06);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc07, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc06, FireFlyOffsetFreeMPC_grad_eq07);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc08, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc07, FireFlyOffsetFreeMPC_grad_eq08);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc09, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc08, FireFlyOffsetFreeMPC_grad_eq09);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc10, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc09, FireFlyOffsetFreeMPC_grad_eq10);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc11, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc10, FireFlyOffsetFreeMPC_grad_eq11);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc12, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc11, FireFlyOffsetFreeMPC_grad_eq12);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc13, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc12, FireFlyOffsetFreeMPC_grad_eq13);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc14, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc13, FireFlyOffsetFreeMPC_grad_eq14);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc15, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc14, FireFlyOffsetFreeMPC_grad_eq15);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc16, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc15, FireFlyOffsetFreeMPC_grad_eq16);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc17, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc16, FireFlyOffsetFreeMPC_grad_eq17);
FireFlyOffsetFreeMPC_LA_DENSE_DIAGZERO_MTVM2_14_17_14(FireFlyOffsetFreeMPC_C01, FireFlyOffsetFreeMPC_dvcc18, FireFlyOffsetFreeMPC_D02, FireFlyOffsetFreeMPC_dvcc17, FireFlyOffsetFreeMPC_grad_eq18);
FireFlyOffsetFreeMPC_LA_DIAGZERO_MTVM_14_14(FireFlyOffsetFreeMPC_D19, FireFlyOffsetFreeMPC_dvcc18, FireFlyOffsetFreeMPC_grad_eq19);
FireFlyOffsetFreeMPC_LA_VSUB_337(FireFlyOffsetFreeMPC_rd, FireFlyOffsetFreeMPC_grad_eq, FireFlyOffsetFreeMPC_rd);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi00, FireFlyOffsetFreeMPC_rd00, FireFlyOffsetFreeMPC_dzcc00);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi01, FireFlyOffsetFreeMPC_rd01, FireFlyOffsetFreeMPC_dzcc01);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi02, FireFlyOffsetFreeMPC_rd02, FireFlyOffsetFreeMPC_dzcc02);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi03, FireFlyOffsetFreeMPC_rd03, FireFlyOffsetFreeMPC_dzcc03);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi04, FireFlyOffsetFreeMPC_rd04, FireFlyOffsetFreeMPC_dzcc04);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi05, FireFlyOffsetFreeMPC_rd05, FireFlyOffsetFreeMPC_dzcc05);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi06, FireFlyOffsetFreeMPC_rd06, FireFlyOffsetFreeMPC_dzcc06);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi07, FireFlyOffsetFreeMPC_rd07, FireFlyOffsetFreeMPC_dzcc07);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi08, FireFlyOffsetFreeMPC_rd08, FireFlyOffsetFreeMPC_dzcc08);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi09, FireFlyOffsetFreeMPC_rd09, FireFlyOffsetFreeMPC_dzcc09);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi10, FireFlyOffsetFreeMPC_rd10, FireFlyOffsetFreeMPC_dzcc10);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi11, FireFlyOffsetFreeMPC_rd11, FireFlyOffsetFreeMPC_dzcc11);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi12, FireFlyOffsetFreeMPC_rd12, FireFlyOffsetFreeMPC_dzcc12);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi13, FireFlyOffsetFreeMPC_rd13, FireFlyOffsetFreeMPC_dzcc13);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi14, FireFlyOffsetFreeMPC_rd14, FireFlyOffsetFreeMPC_dzcc14);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi15, FireFlyOffsetFreeMPC_rd15, FireFlyOffsetFreeMPC_dzcc15);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi16, FireFlyOffsetFreeMPC_rd16, FireFlyOffsetFreeMPC_dzcc16);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi17, FireFlyOffsetFreeMPC_rd17, FireFlyOffsetFreeMPC_dzcc17);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_17(FireFlyOffsetFreeMPC_Phi18, FireFlyOffsetFreeMPC_rd18, FireFlyOffsetFreeMPC_dzcc18);
FireFlyOffsetFreeMPC_LA_DENSE_FORWARDBACKWARDSUB_14(FireFlyOffsetFreeMPC_Phi19, FireFlyOffsetFreeMPC_rd19, FireFlyOffsetFreeMPC_dzcc19);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl00, FireFlyOffsetFreeMPC_slb00, FireFlyOffsetFreeMPC_llbbyslb00, FireFlyOffsetFreeMPC_dzcc00, FireFlyOffsetFreeMPC_lbIdx00, FireFlyOffsetFreeMPC_dllbcc00);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub00, FireFlyOffsetFreeMPC_sub00, FireFlyOffsetFreeMPC_lubbysub00, FireFlyOffsetFreeMPC_dzcc00, FireFlyOffsetFreeMPC_ubIdx00, FireFlyOffsetFreeMPC_dlubcc00);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl01, FireFlyOffsetFreeMPC_slb01, FireFlyOffsetFreeMPC_llbbyslb01, FireFlyOffsetFreeMPC_dzcc01, FireFlyOffsetFreeMPC_lbIdx01, FireFlyOffsetFreeMPC_dllbcc01);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub01, FireFlyOffsetFreeMPC_sub01, FireFlyOffsetFreeMPC_lubbysub01, FireFlyOffsetFreeMPC_dzcc01, FireFlyOffsetFreeMPC_ubIdx01, FireFlyOffsetFreeMPC_dlubcc01);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl02, FireFlyOffsetFreeMPC_slb02, FireFlyOffsetFreeMPC_llbbyslb02, FireFlyOffsetFreeMPC_dzcc02, FireFlyOffsetFreeMPC_lbIdx02, FireFlyOffsetFreeMPC_dllbcc02);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub02, FireFlyOffsetFreeMPC_sub02, FireFlyOffsetFreeMPC_lubbysub02, FireFlyOffsetFreeMPC_dzcc02, FireFlyOffsetFreeMPC_ubIdx02, FireFlyOffsetFreeMPC_dlubcc02);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl03, FireFlyOffsetFreeMPC_slb03, FireFlyOffsetFreeMPC_llbbyslb03, FireFlyOffsetFreeMPC_dzcc03, FireFlyOffsetFreeMPC_lbIdx03, FireFlyOffsetFreeMPC_dllbcc03);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub03, FireFlyOffsetFreeMPC_sub03, FireFlyOffsetFreeMPC_lubbysub03, FireFlyOffsetFreeMPC_dzcc03, FireFlyOffsetFreeMPC_ubIdx03, FireFlyOffsetFreeMPC_dlubcc03);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl04, FireFlyOffsetFreeMPC_slb04, FireFlyOffsetFreeMPC_llbbyslb04, FireFlyOffsetFreeMPC_dzcc04, FireFlyOffsetFreeMPC_lbIdx04, FireFlyOffsetFreeMPC_dllbcc04);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub04, FireFlyOffsetFreeMPC_sub04, FireFlyOffsetFreeMPC_lubbysub04, FireFlyOffsetFreeMPC_dzcc04, FireFlyOffsetFreeMPC_ubIdx04, FireFlyOffsetFreeMPC_dlubcc04);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl05, FireFlyOffsetFreeMPC_slb05, FireFlyOffsetFreeMPC_llbbyslb05, FireFlyOffsetFreeMPC_dzcc05, FireFlyOffsetFreeMPC_lbIdx05, FireFlyOffsetFreeMPC_dllbcc05);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub05, FireFlyOffsetFreeMPC_sub05, FireFlyOffsetFreeMPC_lubbysub05, FireFlyOffsetFreeMPC_dzcc05, FireFlyOffsetFreeMPC_ubIdx05, FireFlyOffsetFreeMPC_dlubcc05);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl06, FireFlyOffsetFreeMPC_slb06, FireFlyOffsetFreeMPC_llbbyslb06, FireFlyOffsetFreeMPC_dzcc06, FireFlyOffsetFreeMPC_lbIdx06, FireFlyOffsetFreeMPC_dllbcc06);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub06, FireFlyOffsetFreeMPC_sub06, FireFlyOffsetFreeMPC_lubbysub06, FireFlyOffsetFreeMPC_dzcc06, FireFlyOffsetFreeMPC_ubIdx06, FireFlyOffsetFreeMPC_dlubcc06);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl07, FireFlyOffsetFreeMPC_slb07, FireFlyOffsetFreeMPC_llbbyslb07, FireFlyOffsetFreeMPC_dzcc07, FireFlyOffsetFreeMPC_lbIdx07, FireFlyOffsetFreeMPC_dllbcc07);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub07, FireFlyOffsetFreeMPC_sub07, FireFlyOffsetFreeMPC_lubbysub07, FireFlyOffsetFreeMPC_dzcc07, FireFlyOffsetFreeMPC_ubIdx07, FireFlyOffsetFreeMPC_dlubcc07);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl08, FireFlyOffsetFreeMPC_slb08, FireFlyOffsetFreeMPC_llbbyslb08, FireFlyOffsetFreeMPC_dzcc08, FireFlyOffsetFreeMPC_lbIdx08, FireFlyOffsetFreeMPC_dllbcc08);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub08, FireFlyOffsetFreeMPC_sub08, FireFlyOffsetFreeMPC_lubbysub08, FireFlyOffsetFreeMPC_dzcc08, FireFlyOffsetFreeMPC_ubIdx08, FireFlyOffsetFreeMPC_dlubcc08);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl09, FireFlyOffsetFreeMPC_slb09, FireFlyOffsetFreeMPC_llbbyslb09, FireFlyOffsetFreeMPC_dzcc09, FireFlyOffsetFreeMPC_lbIdx09, FireFlyOffsetFreeMPC_dllbcc09);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub09, FireFlyOffsetFreeMPC_sub09, FireFlyOffsetFreeMPC_lubbysub09, FireFlyOffsetFreeMPC_dzcc09, FireFlyOffsetFreeMPC_ubIdx09, FireFlyOffsetFreeMPC_dlubcc09);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl10, FireFlyOffsetFreeMPC_slb10, FireFlyOffsetFreeMPC_llbbyslb10, FireFlyOffsetFreeMPC_dzcc10, FireFlyOffsetFreeMPC_lbIdx10, FireFlyOffsetFreeMPC_dllbcc10);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub10, FireFlyOffsetFreeMPC_sub10, FireFlyOffsetFreeMPC_lubbysub10, FireFlyOffsetFreeMPC_dzcc10, FireFlyOffsetFreeMPC_ubIdx10, FireFlyOffsetFreeMPC_dlubcc10);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl11, FireFlyOffsetFreeMPC_slb11, FireFlyOffsetFreeMPC_llbbyslb11, FireFlyOffsetFreeMPC_dzcc11, FireFlyOffsetFreeMPC_lbIdx11, FireFlyOffsetFreeMPC_dllbcc11);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub11, FireFlyOffsetFreeMPC_sub11, FireFlyOffsetFreeMPC_lubbysub11, FireFlyOffsetFreeMPC_dzcc11, FireFlyOffsetFreeMPC_ubIdx11, FireFlyOffsetFreeMPC_dlubcc11);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl12, FireFlyOffsetFreeMPC_slb12, FireFlyOffsetFreeMPC_llbbyslb12, FireFlyOffsetFreeMPC_dzcc12, FireFlyOffsetFreeMPC_lbIdx12, FireFlyOffsetFreeMPC_dllbcc12);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub12, FireFlyOffsetFreeMPC_sub12, FireFlyOffsetFreeMPC_lubbysub12, FireFlyOffsetFreeMPC_dzcc12, FireFlyOffsetFreeMPC_ubIdx12, FireFlyOffsetFreeMPC_dlubcc12);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl13, FireFlyOffsetFreeMPC_slb13, FireFlyOffsetFreeMPC_llbbyslb13, FireFlyOffsetFreeMPC_dzcc13, FireFlyOffsetFreeMPC_lbIdx13, FireFlyOffsetFreeMPC_dllbcc13);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub13, FireFlyOffsetFreeMPC_sub13, FireFlyOffsetFreeMPC_lubbysub13, FireFlyOffsetFreeMPC_dzcc13, FireFlyOffsetFreeMPC_ubIdx13, FireFlyOffsetFreeMPC_dlubcc13);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl14, FireFlyOffsetFreeMPC_slb14, FireFlyOffsetFreeMPC_llbbyslb14, FireFlyOffsetFreeMPC_dzcc14, FireFlyOffsetFreeMPC_lbIdx14, FireFlyOffsetFreeMPC_dllbcc14);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub14, FireFlyOffsetFreeMPC_sub14, FireFlyOffsetFreeMPC_lubbysub14, FireFlyOffsetFreeMPC_dzcc14, FireFlyOffsetFreeMPC_ubIdx14, FireFlyOffsetFreeMPC_dlubcc14);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl15, FireFlyOffsetFreeMPC_slb15, FireFlyOffsetFreeMPC_llbbyslb15, FireFlyOffsetFreeMPC_dzcc15, FireFlyOffsetFreeMPC_lbIdx15, FireFlyOffsetFreeMPC_dllbcc15);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub15, FireFlyOffsetFreeMPC_sub15, FireFlyOffsetFreeMPC_lubbysub15, FireFlyOffsetFreeMPC_dzcc15, FireFlyOffsetFreeMPC_ubIdx15, FireFlyOffsetFreeMPC_dlubcc15);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl16, FireFlyOffsetFreeMPC_slb16, FireFlyOffsetFreeMPC_llbbyslb16, FireFlyOffsetFreeMPC_dzcc16, FireFlyOffsetFreeMPC_lbIdx16, FireFlyOffsetFreeMPC_dllbcc16);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub16, FireFlyOffsetFreeMPC_sub16, FireFlyOffsetFreeMPC_lubbysub16, FireFlyOffsetFreeMPC_dzcc16, FireFlyOffsetFreeMPC_ubIdx16, FireFlyOffsetFreeMPC_dlubcc16);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl17, FireFlyOffsetFreeMPC_slb17, FireFlyOffsetFreeMPC_llbbyslb17, FireFlyOffsetFreeMPC_dzcc17, FireFlyOffsetFreeMPC_lbIdx17, FireFlyOffsetFreeMPC_dllbcc17);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub17, FireFlyOffsetFreeMPC_sub17, FireFlyOffsetFreeMPC_lubbysub17, FireFlyOffsetFreeMPC_dzcc17, FireFlyOffsetFreeMPC_ubIdx17, FireFlyOffsetFreeMPC_dlubcc17);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsl18, FireFlyOffsetFreeMPC_slb18, FireFlyOffsetFreeMPC_llbbyslb18, FireFlyOffsetFreeMPC_dzcc18, FireFlyOffsetFreeMPC_lbIdx18, FireFlyOffsetFreeMPC_dllbcc18);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_6(FireFlyOffsetFreeMPC_ccrhsub18, FireFlyOffsetFreeMPC_sub18, FireFlyOffsetFreeMPC_lubbysub18, FireFlyOffsetFreeMPC_dzcc18, FireFlyOffsetFreeMPC_ubIdx18, FireFlyOffsetFreeMPC_dlubcc18);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTSUB_INDEXED_3(FireFlyOffsetFreeMPC_ccrhsl19, FireFlyOffsetFreeMPC_slb19, FireFlyOffsetFreeMPC_llbbyslb19, FireFlyOffsetFreeMPC_dzcc19, FireFlyOffsetFreeMPC_lbIdx19, FireFlyOffsetFreeMPC_dllbcc19);
FireFlyOffsetFreeMPC_LA_VEC_DIVSUB_MULTADD_INDEXED_3(FireFlyOffsetFreeMPC_ccrhsub19, FireFlyOffsetFreeMPC_sub19, FireFlyOffsetFreeMPC_lubbysub19, FireFlyOffsetFreeMPC_dzcc19, FireFlyOffsetFreeMPC_ubIdx19, FireFlyOffsetFreeMPC_dlubcc19);
FireFlyOffsetFreeMPC_LA_VSUB7_234(FireFlyOffsetFreeMPC_l, FireFlyOffsetFreeMPC_ccrhs, FireFlyOffsetFreeMPC_s, FireFlyOffsetFreeMPC_dl_cc, FireFlyOffsetFreeMPC_ds_cc);
FireFlyOffsetFreeMPC_LA_VADD_337(FireFlyOffsetFreeMPC_dz_cc, FireFlyOffsetFreeMPC_dz_aff);
FireFlyOffsetFreeMPC_LA_VADD_280(FireFlyOffsetFreeMPC_dv_cc, FireFlyOffsetFreeMPC_dv_aff);
FireFlyOffsetFreeMPC_LA_VADD_234(FireFlyOffsetFreeMPC_dl_cc, FireFlyOffsetFreeMPC_dl_aff);
FireFlyOffsetFreeMPC_LA_VADD_234(FireFlyOffsetFreeMPC_ds_cc, FireFlyOffsetFreeMPC_ds_aff);
info->lsit_cc = FireFlyOffsetFreeMPC_LINESEARCH_BACKTRACKING_COMBINED(FireFlyOffsetFreeMPC_z, FireFlyOffsetFreeMPC_v, FireFlyOffsetFreeMPC_l, FireFlyOffsetFreeMPC_s, FireFlyOffsetFreeMPC_dz_cc, FireFlyOffsetFreeMPC_dv_cc, FireFlyOffsetFreeMPC_dl_cc, FireFlyOffsetFreeMPC_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == FireFlyOffsetFreeMPC_NOPROGRESS ){
exitcode = FireFlyOffsetFreeMPC_NOPROGRESS; break;
}
info->it++;
}
output->output_1[0] = FireFlyOffsetFreeMPC_z00[14];
output->output_1[1] = FireFlyOffsetFreeMPC_z00[15];
output->output_1[2] = FireFlyOffsetFreeMPC_z00[16];

#if FireFlyOffsetFreeMPC_SET_TIMING == 1
info->solvetime = FireFlyOffsetFreeMPC_toc(&solvertimer);
#if FireFlyOffsetFreeMPC_SET_PRINTLEVEL > 0 && FireFlyOffsetFreeMPC_SET_TIMING == 1
if( info->it > 1 ){
	PRINTTEXT("Solve time: %5.3f ms (%d iterations)\n\n", info->solvetime*1000, info->it);
} else {
	PRINTTEXT("Solve time: %5.3f ms (%d iteration)\n\n", info->solvetime*1000, info->it);
}
#endif
#else
info->solvetime = -1;
#endif
return exitcode;
}
