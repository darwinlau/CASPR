% Base class for inverse dynamics solvers to inherit from when creating new
% solvers for inverse dynamics
%
% Author        : Darwin LAU
% Created       : 2015
% Description    : The function that must be implemented by the child class
% is the "resolveFunction" function
classdef IDSolverBase < handle
    properties
        model
    end
    
    properties (SetAccess = protected, GetAccess = protected)
        f_previous = []
        active_set = []
    end
    
    methods 
        function id = IDSolverBase(dyn_model)
            id.model = dyn_model;
        end
        
        function [cable_forces, Q_opt, id_exit_type, comp_time, model] = resolve(obj, q, q_d, q_dd, w_ext)
            obj.model.update(q, q_d, q_dd, w_ext);
            start_tic = tic;
            [obj.model.cableForces, Q_opt, id_exit_type] = obj.resolveFunction(obj.model);
            comp_time = toc(start_tic);
            model = obj.model;
            cable_forces = obj.model.cableForces;
        end
    end
    
    methods (Abstract)
        [cable_forces,Q_opt, id_exit_type] = resolveFunction(obj, dynamics);
    end
    
    methods (Static)
        function [A, b] = GetEoMConstraints(dynamics)
            A = -dynamics.L';
            b = dynamics.M*dynamics.q_ddot + dynamics.C + dynamics.G + dynamics.W_e; 
        end
    end
end

