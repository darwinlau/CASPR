% Basic Inverse Dynamics solver for problems in the Quadratic Program form
% This is a well-studied form of inverse dynamics solver for CDPRs.
%
% Author        : Darwin LAU
% Created       : 2016
% Description   : Only a quadratic objective function and linear 
% constraints can be used with this solver. There are multiple types of QP
% solver implementations that can be used with this solver.
classdef LyapunovStaticCompensation < ControllerBase
    
    properties (SetAccess = private)
        id_solver
        Kp
        Kd
    end
    
    methods
        function c = LyapunovStaticCompensation(dyn_model, id_solver, Kp, Kd)
            c@ControllerBase(dyn_model);
            c.id_solver = id_solver;
            c.Kp = Kp;
            c.Kd = Kd;
        end
        
        function [cable_forces] = executeFunction(obj, q, q_d, ~, q_ref, q_ref_d, ~)
            error_comp = obj.Kp * (q_ref - q) + obj.Kd * (q_ref_d - q_d);
            cable_forces = obj.id_solver.resolve(q, zeros(obj.dynModel.numDofs,1), zeros(obj.dynModel.numDofs,1), error_comp);
        end
        
    end
end