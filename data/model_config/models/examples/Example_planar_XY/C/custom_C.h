/*
 *  Template for custom CDPR (Header File)
 */

#ifndef custom_C_h
#define custom_C_h
#include "shrhelp.h"

/* Function declarations */
// Kinematics
EXPORTED_FUNCTION void customCableLengths(double* l, double* q, double* q_dot, double* q_ddot, double* W_e);

// Dynamics
EXPORTED_FUNCTION void customM(double* M, double* q, double* q_dot, double* q_ddot, double* W_e);
EXPORTED_FUNCTION void customC(double* C, double* q, double* q_dot, double* q_ddot, double* W_e);
EXPORTED_FUNCTION void customG(double* G, double* q, double* q_dot, double* q_ddot, double* W_e);
EXPORTED_FUNCTION void customL(double* L, double* q, double* q_dot, double* q_ddot, double* W_e);

#endif