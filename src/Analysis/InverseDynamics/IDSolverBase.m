% Base class for inverse dynamics solvers to inherit from when creating new
% solvers for inverse dynamics
%
% Author        : Darwin LAU
% Created       : 2015
% Description    : The function that must be implemented by the child class
% is the "resolveFunction" function
classdef IDSolverBase < handle
    properties
        model           % The model of the system
    end
    
    properties (SetAccess = protected, GetAccess = protected)
        f_previous = [] % The previous instance of cable forces
        active_set = [] % The previous active set for optimisation based methods
    end
    
    methods 
        % The constructor for the class.
        function id = IDSolverBase(dyn_model)
            id.model = dyn_model;
        end
        
        % Resolves the system kinematics into the next set of cable forces.
        function [forces_active, model, Q_opt, id_exit_type, comp_time] = resolve(obj, q, q_d, q_dd, w_ext)
            obj.model.update(q, q_d, q_dd, w_ext);
            start_tic = tic;
            [forces_active, Q_opt, id_exit_type] = obj.resolveFunction(obj.model);
            comp_time = toc(start_tic);
            obj.model.actuationForces = forces_active;
            model = obj.model;
        end
    end
    
    methods (Abstract)
        % The abstract resolution function to be implemented by concrete
        % implementations of this base class.
        [cable_forces, mode, Q_opt, id_exit_type] = resolveFunction(obj, dynamics);
    end
    
    methods (Static)
        % The equation of motion constraints in linear terms.
        function [A, b] = GetEoMConstraints(dynamics)
            A = [-dynamics.L_active' dynamics.A]; A = round(A,6);
            b = dynamics.M*dynamics.q_ddot + dynamics.C + dynamics.G + dynamics.W_e + dynamics.L_passive' * dynamics.cableForcesPassive; b = round(b,6);
        end
    end
end