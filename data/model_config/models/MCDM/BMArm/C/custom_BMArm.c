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
EXPORTED_FUNCTION void customCableLengths(double* l, struct input_struct st){
    l[0] = st.q_dot[0];    
}

// Mass-inertia matrix (Flat)
EXPORTED_FUNCTION void customM(double* M, struct input_struct st){
    // Sample        
    M[0] = st.q[0];
    M[1] = st.q[1];
    M[2] = st.q[2];
    M[3] = st.q[3];
    M[4] = st.q_dot[0];
    M[5] = st.q_dot[1];
    M[6] = st.q_dot[2];
    M[7] = st.q_dot[3];
}

// Centrifugal and Coriolis vector
EXPORTED_FUNCTION void customC(double* C, struct input_struct st){
    // Sample        
    C[0] = st.q[0]*2 + st.q_dot[0];
    C[1] = st.q[1]*2 + st.q_dot[1];
    C[2] = st.q[2]*2 + st.q_dot[2];
    C[3] = st.q[3]*2 + st.q_dot[3];    
}

// Gravitational vector
EXPORTED_FUNCTION void customG(double* G, struct input_struct st){
    // Sample        
    G[0] = st.q[0]*3 - st.q_dot[0];
    G[1] = st.q[1]*3 - st.q_dot[1];
    G[2] = st.q[2]*3 - st.q_dot[2];
    G[3] = st.q[3]*3 - st.q_dot[3];    
}

// Joint-Cable Jacobian matrix (Flat)
EXPORTED_FUNCTION void customL(double* L, struct input_struct st){
    // Sample        
    L[0] = st.q[0] * st.q_dot[0];
    L[1] = st.q[1] * st.q_dot[1];
    L[2] = st.q[2] * st.q_dot[2];
    L[3] = st.q[3] * st.q_dot[3];    
}

// Needed for mex
void mexFunction( int nlhs, mxArray *plhs[], 
          int nrhs, const mxArray*prhs[] )
{
}