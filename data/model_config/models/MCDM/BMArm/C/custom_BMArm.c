/*
 *  Template for custom CDPR (Source File)
 */

#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <mex.h>
#define EXPORT_FCNS
#include "custom_BMArm.h"

// Cable Lengths
EXPORTED_FUNCTION void customCableLengths(double* l, double* q, double* q_dot, double* q_ddot, double* W_e){
    l[0] = q_dot[0];    
}

// Mass-inertia matrix (Flat)
EXPORTED_FUNCTION void customM(double* M, double* q, double* q_dot, double* q_ddot, double* W_e){
    // Sample        
    M[0] = q[0];
    M[1] = q[1];
    M[2] = q[2];
    M[3] = q[3];
    M[4] = q_dot[0];
    M[5] = q_dot[1];
    M[6] = q_dot[2];
    M[7] = q_dot[3];
}

// Centrifugal and Coriolis vector
EXPORTED_FUNCTION void customC(double* C, double* q, double* q_dot, double* q_ddot, double* W_e){
    // Sample        
    C[0] = q[0]*2 + q_dot[0];
    C[1] = q[1]*2 + q_dot[1];
    C[2] = q[2]*2 + q_dot[2];
    C[3] = q[3]*2 + q_dot[3];    
}

// Gravitational vector
EXPORTED_FUNCTION void customG(double* G, double* q, double* q_dot, double* q_ddot, double* W_e){
    // Sample        
    G[0] = q[0]*3 - q_dot[0];
    G[1] = q[1]*3 - q_dot[1];
    G[2] = q[2]*3 - q_dot[2];
    G[3] = q[3]*3 - q_dot[3];    
}

// Joint-Cable Jacobian matrix (Flat)
EXPORTED_FUNCTION void customL(double* L, double* q, double* q_dot, double* q_ddot, double* W_e){
    // Sample        
    L[0] = q[0] * q_dot[0];
    L[1] = q[1] * q_dot[1];
    L[2] = q[2] * q_dot[2];
    L[3] = q[3] * q_dot[3];    
}

// Needed for mex
void mexFunction( int nlhs, mxArray *plhs[], 
          int nrhs, const mxArray*prhs[] )
{
}