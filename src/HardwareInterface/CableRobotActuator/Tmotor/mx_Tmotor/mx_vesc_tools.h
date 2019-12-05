#ifndef _MX_VESC_TOOLS
#define _MX_VESC_TOOLS

#include "vesc_usb.h"

union vesc_u{
    vescUSB* ptr;
    double val;
};

double vescPtr2double(vescUSB* ptr){
    vesc_u VESC;
    VESC.ptr = ptr;
    return VESC.val;
}

vescUSB* double2vescPtr(double val){
    vesc_u VESC;
    VESC.val = val;
    return VESC.ptr;
}

char* sensor_fields[] = {
    "v_in",
    "pid_pos",
    "current_motor",
    "fault_code"};
static int n_sensor_fields = sizeof(sensor_fields) / sizeof(sensor_fields[0]);

char* config_fields[] = {
    "l_current_max",
    "l_current_min",
    "l_in_current_max",
    "l_in_current_min",
    "l_abs_current_max",
    "l_min_erpm",
    "l_max_erpm",
    "s_pid_kp",
    "s_pid_ki",
    "s_pid_kd",
    "s_pid_kd_filter",
    "s_pid_min_erpm",
    "s_pid_allow_braking",
    "p_pid_kp",
    "p_pid_ki",
    "p_pid_kd",
    "p_pid_kd_filter",
    "p_pid_ang_div"};
static int n_config_fields = sizeof(config_fields) / sizeof(config_fields[0]);

#endif