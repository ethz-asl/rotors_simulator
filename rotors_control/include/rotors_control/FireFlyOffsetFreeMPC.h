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
#ifndef __FireFlyOffsetFreeMPC_H__
#define __FireFlyOffsetFreeMPC_H__


// Generated on: 11-Feb-2015 10:24:44
#ifdef __cplusplus
 extern "C" {
#endif
/* DATA TYPE ------------------------------------------------------------*/
typedef double FireFlyOffsetFreeMPC_FLOAT;


/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef FireFlyOffsetFreeMPC_SET_PRINTLEVEL
#define FireFlyOffsetFreeMPC_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef FireFlyOffsetFreeMPC_SET_TIMING
#define FireFlyOffsetFreeMPC_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define FireFlyOffsetFreeMPC_SET_MAXIT         (30)	

/* scaling factor of line search (affine direction) */
#define FireFlyOffsetFreeMPC_SET_LS_SCALE_AFF  (0.9)      

/* scaling factor of line search (combined direction) */
#define FireFlyOffsetFreeMPC_SET_LS_SCALE      (0.95)  

/* minimum required step size in each iteration */
#define FireFlyOffsetFreeMPC_SET_LS_MINSTEP    (1E-08)

/* maximum step size (combined direction) */
#define FireFlyOffsetFreeMPC_SET_LS_MAXSTEP    (0.995)

/* desired relative duality gap */
#define FireFlyOffsetFreeMPC_SET_ACC_RDGAP     (0.0001)

/* desired maximum residual on equality constraints */
#define FireFlyOffsetFreeMPC_SET_ACC_RESEQ     (1E-06)

/* desired maximum residual on inequality constraints */
#define FireFlyOffsetFreeMPC_SET_ACC_RESINEQ   (1E-06)

/* desired maximum violation of complementarity */
#define FireFlyOffsetFreeMPC_SET_ACC_KKTCOMPL  (1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define FireFlyOffsetFreeMPC_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define FireFlyOffsetFreeMPC_MAXITREACHED (0)

/* no progress in line search possible */
#define FireFlyOffsetFreeMPC_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct FireFlyOffsetFreeMPC_params
{
    /* vector of size 28 */
    FireFlyOffsetFreeMPC_FLOAT z1[28];
    /* matrix of size [17 x 17] (column major format) */
    FireFlyOffsetFreeMPC_FLOAT H[289];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_1[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_2[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_3[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_4[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_5[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_6[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_7[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_8[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_9[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_10[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_11[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_12[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_13[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_14[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_15[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_16[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_17[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_18[17];
    /* vector of size 17 */
    FireFlyOffsetFreeMPC_FLOAT f_19[17];
    /* vector of size 14 */
    FireFlyOffsetFreeMPC_FLOAT f_N[14];
    /* matrix of size [14 x 14] (column major format) */
    FireFlyOffsetFreeMPC_FLOAT H_N[196];
} FireFlyOffsetFreeMPC_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct FireFlyOffsetFreeMPC_output
{
    /* vector of size 3 */
    FireFlyOffsetFreeMPC_FLOAT output_1[3];
} FireFlyOffsetFreeMPC_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct FireFlyOffsetFreeMPC_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    FireFlyOffsetFreeMPC_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    FireFlyOffsetFreeMPC_FLOAT res_ineq;

    /* primal objective */
    FireFlyOffsetFreeMPC_FLOAT pobj;	
	
    /* dual objective */
    FireFlyOffsetFreeMPC_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    FireFlyOffsetFreeMPC_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    FireFlyOffsetFreeMPC_FLOAT rdgap;		

    /* duality measure */
    FireFlyOffsetFreeMPC_FLOAT mu;

	/* duality measure (after affine step) */
    FireFlyOffsetFreeMPC_FLOAT mu_aff;
	
    /* centering parameter */
    FireFlyOffsetFreeMPC_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    FireFlyOffsetFreeMPC_FLOAT step_aff;
    
    /* step size (combined direction) */
    FireFlyOffsetFreeMPC_FLOAT step_cc;    

	/* solvertime */
	FireFlyOffsetFreeMPC_FLOAT solvetime;   

} FireFlyOffsetFreeMPC_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int FireFlyOffsetFreeMPC_solve(FireFlyOffsetFreeMPC_params* params, FireFlyOffsetFreeMPC_output* output, FireFlyOffsetFreeMPC_info* info);


#ifdef __cplusplus
 } 
#endif
#endif
