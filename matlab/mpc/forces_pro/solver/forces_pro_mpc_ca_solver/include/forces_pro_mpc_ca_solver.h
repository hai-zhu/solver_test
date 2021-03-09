/*
forces_pro_mpc_ca_solver : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCESPRO v4.2.0 on Monday, March 8, 2021 at 11:45:02 PM */
#ifndef forces_pro_mpc_ca_solver_H
#define forces_pro_mpc_ca_solver_H

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif


#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double forces_pro_mpc_ca_solver_float;
typedef double forces_pro_mpc_ca_solver_callback_float;

typedef double forces_pro_mpc_ca_solverinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_forces_pro_mpc_ca_solver
#define MISRA_C_forces_pro_mpc_ca_solver (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_forces_pro_mpc_ca_solver
#define RESTRICT_CODE_forces_pro_mpc_ca_solver (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_forces_pro_mpc_ca_solver
#define SET_PRINTLEVEL_forces_pro_mpc_ca_solver    (0)
#endif

/* timing */
#ifndef SET_TIMING_forces_pro_mpc_ca_solver
#define SET_TIMING_forces_pro_mpc_ca_solver    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_forces_pro_mpc_ca_solver			(300)	

/* scaling factor of line search (FTB rule) */
#define SET_FLS_SCALE_forces_pro_mpc_ca_solver		(forces_pro_mpc_ca_solver_float)(0.99)      

/* maximum number of supported elements in the filter */
#define MAX_FILTER_SIZE_forces_pro_mpc_ca_solver	(300) 

/* maximum number of supported elements in the filter */
#define MAX_SOC_IT_forces_pro_mpc_ca_solver			(4) 

/* desired relative duality gap */
#define SET_ACC_RDGAP_forces_pro_mpc_ca_solver		(forces_pro_mpc_ca_solver_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_forces_pro_mpc_ca_solver		(forces_pro_mpc_ca_solver_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_forces_pro_mpc_ca_solver	(forces_pro_mpc_ca_solver_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_forces_pro_mpc_ca_solver	(forces_pro_mpc_ca_solver_float)(1E-06)


/* SOLVER RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_forces_pro_mpc_ca_solver      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_forces_pro_mpc_ca_solver (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_forces_pro_mpc_ca_solver   (2)

/* wrong number of inequalities error */
#define INVALID_NUM_INEQ_ERROR_forces_pro_mpc_ca_solver  (-4)

/* factorization error */
#define FACTORIZATION_ERROR_forces_pro_mpc_ca_solver   (-5)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_forces_pro_mpc_ca_solver  (-6)

/* no progress in method possible */
#define NOPROGRESS_forces_pro_mpc_ca_solver   (-7)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_forces_pro_mpc_ca_solver   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_forces_pro_mpc_ca_solver   (-12)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_forces_pro_mpc_ca_solver  (-100)

/* INTEGRATORS RETURN CODE ------------*/
/* Integrator ran successfully */
#define INTEGRATOR_SUCCESS (11)
/* Number of steps set by user exceeds maximum number of steps allowed */
#define INTEGRATOR_MAXSTEPS_EXCEEDED (12)



/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 140 */
    forces_pro_mpc_ca_solver_float x0[140];

    /* vector of size 4 */
    forces_pro_mpc_ca_solver_float xinit[4];

    /* vector of size 380 */
    forces_pro_mpc_ca_solver_float all_parameters[380];


} forces_pro_mpc_ca_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x01[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x02[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x03[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x04[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x05[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x06[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x07[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x08[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x09[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x10[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x11[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x12[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x13[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x14[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x15[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x16[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x17[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x18[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x19[7];

    /* vector of size 7 */
    forces_pro_mpc_ca_solver_float x20[7];


} forces_pro_mpc_ca_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    forces_pro_mpc_ca_solver_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    forces_pro_mpc_ca_solver_float res_ineq;

	/* norm of stationarity condition */
    forces_pro_mpc_ca_solver_float rsnorm;

	/* max of all complementarity violations */
    forces_pro_mpc_ca_solver_float rcompnorm;

    /* primal objective */
    forces_pro_mpc_ca_solver_float pobj;	
	
    /* dual objective */
    forces_pro_mpc_ca_solver_float dobj;	

    /* duality gap := pobj - dobj */
    forces_pro_mpc_ca_solver_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    forces_pro_mpc_ca_solver_float rdgap;		

    /* duality measure */
    forces_pro_mpc_ca_solver_float mu;

	/* duality measure (after affine step) */
    forces_pro_mpc_ca_solver_float mu_aff;
	
    /* centering parameter */
    forces_pro_mpc_ca_solver_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    forces_pro_mpc_ca_solver_float step_aff;
    
    /* step size (combined direction) */
    forces_pro_mpc_ca_solver_float step_cc;    

	/* solvertime */
	forces_pro_mpc_ca_solver_float solvetime;   

	/* time spent in function evaluations */
	forces_pro_mpc_ca_solver_float fevalstime;  


} forces_pro_mpc_ca_solver_info;







/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Monday, March 8, 2021 11:45:03 PM */
/* User License expires on: (UTC) Sunday, March 28, 2021 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Sunday, March 28, 2021 10:00:00 PM (approx.) */
/* Solver Generation Request Id: 39e6038f-3f5f-4d6d-ae34-5a9c777290a2 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*forces_pro_mpc_ca_solver_extfunc)(forces_pro_mpc_ca_solver_float* x, forces_pro_mpc_ca_solver_float* y, forces_pro_mpc_ca_solver_float* lambda, forces_pro_mpc_ca_solver_float* params, forces_pro_mpc_ca_solver_float* pobj, forces_pro_mpc_ca_solver_float* g, forces_pro_mpc_ca_solver_float* c, forces_pro_mpc_ca_solver_float* Jeq, forces_pro_mpc_ca_solver_float* h, forces_pro_mpc_ca_solver_float* Jineq, forces_pro_mpc_ca_solver_float* H, solver_int32_default stage, solver_int32_default iterations, solver_int32_default threadID);

extern solver_int32_default forces_pro_mpc_ca_solver_solve(forces_pro_mpc_ca_solver_params *params, forces_pro_mpc_ca_solver_output *output, forces_pro_mpc_ca_solver_info *info, FILE *fs, forces_pro_mpc_ca_solver_extfunc evalextfunctions_forces_pro_mpc_ca_solver);	









#ifdef __cplusplus
}
#endif

#endif
