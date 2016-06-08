% An efficient inverse dynamics solver for CDPR systems.
%
% Please cite the following paper when using this algorithm:
% A. Pott, "An Improved Force Distribution Algorithm for Over-Constrained
% Cable-Driven Parallel Robots", Proceedings of the 6th International
% Workshop on Computational Kinematics (CK2013), pp. 139-146, 2014.
%
% If the puncture method is also utilised please cite:
% K. M�ller and C. Reichert and T. Bruckmann. 
% "Analysis of a real-time capable cable force computation method." 
% In Cable-Driven Parallel Robots, pp. 227-238. Springer International 
% Publishing, 2015.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : 
classdef IDSolverClosedForm < IDSolverBase
    properties (SetAccess = private)
        cf_solver_type
        options
    end
    methods
        % The constructor for this class
        function id = IDSolverClosedForm(model,cf_solver_type)
            id@IDSolverBase(model);
            id.cf_solver_type = cf_solver_type;
        end
        
        % The implementation of the resolveFunction.
        function [cable_forces,Q_opt, id_exit_type] = resolveFunction(obj, dynamics)            
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A_eq, b_eq] = IDSolverBase.GetEoMConstraints(dynamics);  
            % Form the lower and upper bound force constraints
            fmin = dynamics.forcesMin;
            fmax = dynamics.forcesMax;

            switch (obj.cf_solver_type)
                case ID_CF_SolverType.CLOSED_FORM
                    [cable_forces, id_exit_type] = id_cf_cfm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.IMPROVED_CLOSED_FORM
                    [cable_forces, id_exit_type] = id_cf_icfm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.PUNCTURE_METHOD
                    [cable_forces, id_exit_type] = id_cf_pm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.IMPROVED_PUNCTURE_METHOD
                    [cable_forces, id_exit_type] = id_cf_ipm(A_eq, b_eq, fmin, fmax);
                otherwise
                    error('ID_CF_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                cable_forces = dynamics.cableModel.FORCES_INVALID;
                Q_opt = Inf;
            else
                Q_opt = norm(cable_forces);
            end            
            
            obj.f_previous = cable_forces;
        end
    end
    
end

