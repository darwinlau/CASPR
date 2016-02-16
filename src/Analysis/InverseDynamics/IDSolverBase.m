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
        
        function [cable_forces, Q_opt, id_exit_type, comp_time, model] = resolve(obj, q, q_d, q_dd)
            obj.model.update(q, q_d, q_dd);
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
            b = dynamics.M*dynamics.q_ddot + dynamics.C + dynamics.G; 
        end
        
%         function exit_type = DisplayOptiToolboxError(exit_flag)
%             switch exit_flag
%                 case -1
%                     fprintf('Infeasible\n');
%                     exit_type = IDExitType.INFEASIBLE;
%                 case 0
%                     fprintf('Max limit reached\n');
%                     exit_type = IDExitType.LIMIT_REACHED;
%                 case -3
%                     fprintf('Solver specific error\n');
%                     exit_type = IDExitType.SOLVER_SPECIFIC_ERROR;
%                 otherwise
%                     fprintf('Other error : Code %d\n', exit_flag);
%                     exit_type = IDExitType.OTHER_ERROR;
%             end
    end
end

