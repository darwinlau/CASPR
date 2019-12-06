/*
 * input is a char array pointing to the VESC USB device. e.g. /dev/ttyACM0.
 * output is the pointer to the VESC object, carried in a scalar double.
 *
 */

#include <mex.h>
#include <string.h>
#include "vesc_usb.h"
#include "mx_vesc_tools.h"
#include <unistd.h>


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    
    //////////////////// Get the command string ////////////////////
    char cmd[64];
     
    // Check there is a second input, which should be the class instance handle
    if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd))) {
        mexErrMsgTxt("The first input should be a command string less than 64 characters long.\n");
    }
    
          
    //////////////////// new vescUSB instance ////////////////////
    // returns a vescUSB pointer as a double
    if (!strcmp("new", cmd)) {
        // Check parameters
        if (nlhs != 1) mexErrMsgTxt("New: One output expected.");
        
        // vescUSB ptr as a double
        vescUSB* _VESC = new vescUSB;
        plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
        *(mxGetPr(plhs[0])) = vescPtr2double(_VESC);
        return; //exit
    }       
    
    //////////////////// other functions ////////////////////
    // other functions expect the second argument as a vescUSB pointer
    if (nrhs < 2) {
        mexErrMsgTxt("The second input to functions other than \"new\" should be a vescUSB ptr (double).\n");
        return;
    }
    double* vescPtrDouble = mxGetPr(prhs[1]);
    vescUSB* vescPtr = double2vescPtr(*vescPtrDouble); // get ptr to vescUSB
    
    //////////////////// open serial dev ////////////////////
    // opens communications with the vescUSB instance
    if (!strcmp("open", cmd)) {
        if (nrhs != 3) {
            mexErrMsgTxt("The third input to \"open\" should be a serial device char array.\n");
            return;
        }
        
        char dev[50];
        mxGetString(prhs[2], dev, sizeof(dev));
        mexPrintf("Closing device...\n");
        vescPtr -> comm_close();
        mexPrintf("Opening device...\n");
        vescPtr -> comm_init(dev);    
        vescPtr -> receive_packet();
        for (int i = 0; i < 10; i++) {
            vescPtr -> bldc_interface_set_current(0.0);
        }
        return;
    }
    //////////////////// close serial dev ////////////////////
    else if (!strcmp("close", cmd)) {
        mexPrintf("Closing device...\n");
        for (int i = 0; i < 10; i++) {
            vescPtr -> bldc_interface_set_current(0.0);
        }
        vescPtr -> comm_close();
        return;
    }
    //////////////////// delete serial dev ////////////////////
    else if (!strcmp("delete", cmd)) {
        // expects a vescUSB ptr (carried via a double)
        // Destroy the C++ object
        mexPrintf("Closing device...\n");
        for (int i = 0; i < 10; i++) {
            vescPtr -> bldc_interface_set_current(0.0);
        }
        vescPtr -> comm_close();
        mexPrintf("Deleting device...\n");
        delete vescPtr;
       
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    //////////////////// get_sensors ////////////////////
    else if (!strcmp("get_sensors", cmd)) {
        if (nlhs != 1) mexErrMsgTxt("get_sensors: one outputs expected.");
        
        vescPtr -> values_read = false;
        
        int n_reads = 0;
        
        while (!(vescPtr -> values_read) && (n_reads++ < 1000)) {
            vescPtr -> bldc_interface_get_values();
            vescPtr -> receive_packet();
            if (n_reads >= 1000) {
                mexPrintf("No response from device.\n");
                mexErrMsgTxt("***\n");
            }
        }
                
        mwSize dims[2] = {1, 1};
        plhs[0] = mxCreateStructArray(2, dims, n_sensor_fields, sensor_fields);
        
        mxArray* sensor_val[n_sensor_fields];
        for (int i = 0; i < n_sensor_fields; i++) {
            sensor_val[i] = mxCreateDoubleMatrix(1, 1, mxREAL);
        }
        
        *mxGetPr(sensor_val[0]) = (double) (vescPtr -> values.v_in);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 0, sensor_val[0]);
        *mxGetPr(sensor_val[1]) = (double) (vescPtr -> values.pid_pos);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 1, sensor_val[1]);
        //*mxGetPr(sensor_val[2]) = (double) (vescPtr -> values.current_motor);
        *mxGetPr(sensor_val[2]) = (double) (vescPtr -> values.iq);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 2, sensor_val[2]);
        *mxGetPr(sensor_val[3]) = (double) (vescPtr -> values.fault_code);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 3, sensor_val[3]);
        //usleep(100); // ADDED TO ALLEVIATE 100% CPU
        return;
    }
    //////////////////// get_config ////////////////////
    else if (!strcmp("get_config", cmd)) {
        if (nlhs != 1) mexErrMsgTxt("get_config: 1 output expected.");
        
        vescPtr -> mcconf_read = false;
        
        mexPrintf("Reading mcconfig...\n");
        
        int n_reads = 0;
        
        vescPtr -> receive_packet();
        
        while (!(vescPtr -> mcconf_read) && (n_reads++ < 10000)) {
            vescPtr -> bldc_interface_get_mcconf(); 
            usleep(100);
            vescPtr -> receive_packet();
            if (n_reads >= 10000) {
                mexPrintf("No response from device.\n");
                mexErrMsgTxt("***\n");
            }
        }
            
        mwSize dims[2] = {1, 1};
        plhs[0] = mxCreateStructArray(2, dims, n_config_fields, config_fields);

        mxArray* config_val[n_config_fields];
        for (int i = 0; i < n_config_fields; i++) {
            config_val[i] = mxCreateDoubleMatrix(1, 1, mxREAL);
        }
        
        *mxGetPr(config_val[0]) = (double) (vescPtr -> mcconf.l_current_max);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 0, config_val[0]);
        
        *mxGetPr(config_val[1]) = (double) (vescPtr -> mcconf.l_current_min);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 1, config_val[1]);
        
        *mxGetPr(config_val[2]) = (double) (vescPtr -> mcconf.l_in_current_max);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 2, config_val[2]);
        
        *mxGetPr(config_val[3]) = (double) (vescPtr -> mcconf.l_in_current_min);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 3, config_val[3]);
        
        *mxGetPr(config_val[4]) = (double) (vescPtr -> mcconf.l_abs_current_max);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 4, config_val[4]);
        
        *mxGetPr(config_val[5]) = (double) (vescPtr -> mcconf.l_min_erpm);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 5, config_val[5]);
        
        *mxGetPr(config_val[6]) = (double) (vescPtr -> mcconf.l_max_erpm);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 6, config_val[6]);
        
        *mxGetPr(config_val[7]) = (double) (vescPtr -> mcconf.s_pid_kp);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 7, config_val[7]);
        
        *mxGetPr(config_val[8]) = (double) (vescPtr -> mcconf.s_pid_ki);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 8, config_val[8]);
        
        *mxGetPr(config_val[9]) = (double) (vescPtr -> mcconf.s_pid_kd);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 9, config_val[9]);
        
        *mxGetPr(config_val[10]) = (double) (vescPtr -> mcconf.s_pid_kd_filter);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 10, config_val[10]);
        
        *mxGetPr(config_val[11]) = (double) (vescPtr -> mcconf.s_pid_min_erpm);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 11, config_val[11]);
        
        *mxGetPr(config_val[12]) = (double) (vescPtr -> mcconf.s_pid_allow_braking);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 12, config_val[12]);
        
        *mxGetPr(config_val[13]) = (double) (vescPtr -> mcconf.p_pid_kp);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 13, config_val[13]);
        
        *mxGetPr(config_val[14]) = (double) (vescPtr -> mcconf.p_pid_ki);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 14, config_val[14]);
        
        *mxGetPr(config_val[15]) = (double) (vescPtr -> mcconf.p_pid_kd);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 15, config_val[15]);
        
        *mxGetPr(config_val[16]) = (double) (vescPtr -> mcconf.p_pid_kd_filter);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 16, config_val[16]);
        
        *mxGetPr(config_val[17]) = (double) (vescPtr -> mcconf.p_pid_ang_div);
        mxSetFieldByNumber(plhs[0], (mwIndex) 0, 17, config_val[17]);
        
        mexPrintf("Read successful.\n");
        
        return;
    }
    //////////////////// send_current ////////////////////
    else if (!strcmp("send_current", cmd)) {
        if (nlhs != 0) mexErrMsgTxt("send_current: no output expected.");
        if (nrhs != 3) {
            mexErrMsgTxt("The third input to \"send_current\" should be a current cmd.\n");
            return;
        }
        double _curr_cmd = (double) *mxGetPr(prhs[2]);
        vescPtr -> bldc_interface_set_current(_curr_cmd);
        
        return;
    }
    //////////////////// unknown cmd ////////////////////
    else {
        mexErrMsgTxt("Unknown command.");
        return;
    }
        
    return;
}