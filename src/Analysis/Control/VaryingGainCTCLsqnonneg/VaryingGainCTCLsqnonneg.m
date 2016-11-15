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
        function [cable_force_active, result_model] = executeFunction(obj, q, q_d, ~, q_ref, q_ref_d, q_ref_dd, ~)
            % following terms are (joint to cable) Jacobian transpose
            % inertia matrix, Coriolis and centrifugal, gravitational (all in joint space)
            L_T = (obj.dynModel.L)';
            M = obj.dynModel.M;
            C = obj.dynModel.C;
            G = obj.dynModel.G;
            % minimum values for cable forces, proportional gains,
            % differential gains as well as the decision variables
            fmin = obj.dynModel.cableForcesActiveMin;
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
            
            C_ = [-L_T M*Eq M*Eq_dot];
            d_ = G + C + M*q_ref_dd - C_*x_min;
            
            % use least square non-negtive algorithm provided by matlab
            x = lsqnonneg(C_,d_);
            
            % extract actual control force
            Cable_num = obj.dynModel.numCables;
            cable_force_active = x(1:Cable_num) + fmin;
            
            
            obj.dynModel.cableForces = cable_force_active;
            result_model = obj.dynModel;
        end
    end
end
