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
        id_solver       % The inverse dynamics solver
        Kp              % The proportional position gain
        Kd              % The proportional derivative gain
        f_prev          % Stors the last feasible force command
    end

    methods
        % A constructor for a computed torque controller.
        function c = ComputedTorqueController(dyn_model, id_solver, Kp, Kd)
            c@ControllerBase(dyn_model);
            c.id_solver = id_solver;
            c.Kp = Kp;
            c.Kd = Kd;
            c.f_prev = zeros(dyn_model.numActuatorsActive, 1);
        end

        % The implementation of the abstract executeFunction for the
        % controller class.
        function [f_active, result_model, exit_flag] = executeFunction(obj, q, q_d, ~, q_ref, q_ref_d, q_ref_dd, ~)
            q_ddot_cmd = q_ref_dd + obj.Kp * (q_ref - q) + obj.Kd * (q_ref_d - q_d);
            % ID solver is also responsible for generating the combined
            % actuating forces and updating the model with the forces
            [f_active, result_model, ~, id_exit_type] = obj.id_solver.resolve(q, q_d, q_ddot_cmd, zeros(obj.dynModel.numDofs,1));
 
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                f_active = obj.f_prev;
            else
                obj.f_prev = f_active;
            end
            
            exit_flag = obj.exitTypeConversion(id_exit_type);
        end
        
        
        
        % Function that accepts a vector of cable stiffness and produce a
        % vector of length offsets to produced the force commands
        function delta_l_vec = offsetGeneration(obj, stiffness_vec)
            delta_l_vec = -obj.f_prev./stiffness_vec;
        end
    end
end
