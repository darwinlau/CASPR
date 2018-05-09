% Computed torque controller for CDPRs using the inverse dynamics resolve
% from the library.
%
% Please cite the following paper when using this algorithm:
% A. B. Alp, and S. K. Agrawal, "Cable Suspended Robots: Design, Planning
% and Control", in Proc. IEEE Int. Conf. Robot. Autom., pp. 4275-4280,
% 2002.
% 
% Author        : Chen SONG
% Created       : 2016
% Description    : 
%    Based on a computed torque controller, a non-negtive least square
%    optimization algorithm is used to solve for the cable forces and
%    control gains, subject to that all cable forces and control gains are
%    larger than given lower bounds (to guarantee canle feasibility, 
%    controller stability as well as convergence speed). The optimization 
%    algorithm is chosen as the one called lsqnonneg offered by Matlab. 
%    Since gains are part of the decision variables in optimization, their 
%    values will change to minimize the objective function (equivalently 
%    try to make the equation of motion hold for system dynamics).

classdef VaryingGainCTCLsqnonneg < ControllerBase

    properties (SetAccess = private)
        kp_min              % The minimum proportional position gain (roll vec)
        kd_min              % The minimum proportional derivative gain (roll vec)
    end

    methods
        % A constructor for the controller.
        function c = VaryingGainCTCLsqnonneg(dyn_model, kpmin, kdmin)
            c@ControllerBase(dyn_model);
            c.kp_min = kpmin;
            c.kd_min = kdmin;
        end

        % The implementation of the abstract executeFunction for the
        % controller class.
        function [f_active, result_model, exit_type] = executeFunction(obj, q, q_d, ~, q_ref, q_ref_d, q_ref_dd, ~)
            % following terms are (joint to cable) Jacobian transpose
            % inertia matrix, Coriolis and centrifugal, gravitational (all in joint space)
            L_T = (obj.dynModel.L)';
            A   =   obj.dynModel.A;
            M   = obj.dynModel.M;
            C   = obj.dynModel.C;
            G   = obj.dynModel.G;
            % minimum values for cable forces, proportional gains,
            % differential gains as well as the decision variables
            fmin = obj.dynModel.actuationForcesMin;
            kpmin = obj.kp_min;
            kdmin = obj.kd_min;
            x_min = [fmin;
                     kpmin;
                     kdmin];
            % error and error_dot of joint space coordinate
            Eq = diag(q - q_ref);
            Eq_dot = diag(q_d - q_ref_d);
            
            % solve a non-negtive least square problem in the form of:
            % min ||C_*(x+x_min) - d_||
            % s.t. x >= 0
            % where decision variable x = [delta_f' delta_kp' delta_kd']'
            
            C_ = [-L_T A M*Eq M*Eq_dot];
            d_ = G + C + M*q_ref_dd - C_*x_min;
            
            % use least square non-negtive algorithm provided by matlab
            [x, ~, ~, exitflag] = lsqnonneg(C_,d_);
            
            % extract actual control force
            actuator_num = obj.dynModel.numActuatorsActive;
            f_active = x(1:actuator_num) + fmin;
            
            
            obj.dynModel.actuationForces = f_active;
            result_model = obj.dynModel;
            
            switch exitflag
                case 1
                    exit_type = ControllerExitType.NO_ERROR;
                case 0
                    CASPR_log.Info('Controller - Max iteration limit reached');
                    exit_type = ControllerExitType.ITERATION_LIMIT_REACHED;
                otherwise
                    CASPR_log.Info('Controller - This is not expected, need further investigation');
                    exit_type = ControllerExitType.SOLVER_SPECIFIC_ERROR;
            end
        end
    end
end
