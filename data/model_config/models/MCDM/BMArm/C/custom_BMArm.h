/*
 *  Template for custom CDPR (Header File)
 */

#ifndef custom_BMArm_h
#define custom_BMArm_h
#include "shrhelp.h"

struct input_struct {
    double* q;
    double* q_dot;
    double* q_ddot;
    double* W_e;
};

/* Function declarations */
// Kinematics
EXPORTED_FUNCTION void customCableLengths(double* l, struct input_struct);

// Dynamics
EXPORTED_FUNCTION void customM(double* M, struct input_struct);
EXPORTED_FUNCTION void customC(double* C, struct input_struct);
EXPORTED_FUNCTION void customG(double* G, struct input_struct);
EXPORTED_FUNCTION void customL(double* L, struct input_struct);

#endif