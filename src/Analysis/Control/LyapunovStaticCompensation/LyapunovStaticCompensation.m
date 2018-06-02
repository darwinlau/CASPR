% Lyapunov-based control law with gravity compensation 
%
% Please cite the following paper when using this for multilink cable
% robots:
% A. B. Alp, and S. K. Agrawal, "Cable Suspended Robots: Design, Planning
% and Control", in Proc. IEEE Int. Conf. Robot. Autom., pp. 4275-4280,
% 2002.
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    This version assumes that Kp and Kd are constant values set by the
%    constructor.
classdef LyapunovStaticCompensation < ControllerBase
    
    properties (SetAccess = private)
        id_solver       % The inverse dynamics solver
        Kp              % The proportional position gain
        Kd              % The proportional derivative gain
    end
    
    methods
        % A constructor for a computed torque controller.
        function c = LyapunovStaticCompensation(dyn_model, id_solver, Kp, Kd)
            c@ControllerBase(dyn_model);
            c.id_solver = id_solver;
            c.Kp = Kp;
            c.Kd = Kd;
        end
        
        % The implementation of the abstract executeFunction for the
        % controller class.
        function [f_active, result_model, exit_flag] = executeFunction(obj, q, q_d, ~, q_ref, q_ref_d, ~, ~)
            error_comp = obj.Kp * (q_ref - q) + obj.Kd * (q_ref_d - q_d);
            [f_active, result_model, ~, id_exit_type] = obj.id_solver.resolve(q, zeros(obj.dynModel.numDofs,1), zeros(obj.dynModel.numDofs,1), error_comp);
            exit_flag = obj.exitTypeConversion(id_exit_type);
        end
    end
end