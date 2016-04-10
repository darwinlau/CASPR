% Computed torque controller for CDPRs using the inverse dynamics resolve
% from the library.
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    This version assumes that Kp and Kd are constant values. The method of
%    resolving the inverse dynamics is generic by specifying the id_solver.
classdef ComputedTorqueController < ControllerBase
    
    properties (SetAccess = private)
        id_solver
        Kp
        Kd
    end
    
    methods
        function c = ComputedTorqueController(dyn_model, id_solver, Kp, Kd)
            c@ControllerBase(dyn_model);
            c.id_solver = id_solver;
            c.Kp = Kp;
            c.Kd = Kd;
        end
        
        function [cable_forces] = executeFunction(obj, q, q_d, ~, q_ref, q_ref_d, q_ref_dd,~)
            q_ddot_cmd = q_ref_dd + obj.Kp * (q_ref - q) + obj.Kd * (q_ref_d - q_d);
            cable_forces = obj.id_solver.resolve(q, q_d, q_ddot_cmd, zeros(obj.dynModel.numDofs,1));
        end
        
    end
end