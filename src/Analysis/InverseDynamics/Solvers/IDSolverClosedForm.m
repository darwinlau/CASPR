% An efficient inverse dynamics solver for CDPR systems.
%
% Please cite the following paper when using this algorithm:
% A. Pott, "An Improved Force Distribution Algorithm for Over-Constrained
% Cable-Driven Parallel Robots", Proceedings of the 6th International
% Workshop on Computational Kinematics (CK2013), pp. 139-146, 2014.
%
% If the puncture method is also utilised please cite:
% K. Muller and C. Reichert and T. Bruckmann.
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
        x_fixed
    end
    
    methods
        % The constructor for this class
        function id = IDSolverClosedForm(model, cf_solver_type)
            id@IDSolverBase(model);
            id.cf_solver_type = cf_solver_type;
            id.active_set = true(model.numCables,1);
            id.x_fixed = zeros(model.numCables,1);
        end

        % The implementation of the resolveFunction.
        function [cable_forces,Q_opt, id_exit_type] = resolveFunction(obj, dynamics)
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + w_{ext} = -L_active^T f_active - L_passive^T f_passive (constraint)
            [A_eq, b_eq] = IDSolverBase.GetEoMConstraints(dynamics);
            % Form the lower and upper bound force constraints
            fmin = dynamics.actuationForcesMin;
            fmax = dynamics.actuationForcesMax;

            switch (obj.cf_solver_type)
                case ID_CF_SolverType.CLOSED_FORM
                    [cable_forces, id_exit_type] = id_cf_cfm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.IMPROVED_CLOSED_FORM
                    [cable_forces, id_exit_type] = id_cf_icfm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.ALTERNATE_IMPROVED_CLOSED_FORM
                    [cable_forces, id_exit_type] = id_cf_aicfm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.WARM_IMPROVED_CLOSED_FORM
                    [cable_forces, id_exit_type,obj.active_set,obj.x_fixed] = id_cf_wicfm(A_eq, b_eq, fmin, fmax, obj.active_set,obj.x_fixed);
                case ID_CF_SolverType.PUNCTURE_METHOD
                    [cable_forces, id_exit_type] = id_cf_pm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.IMPROVED_PUNCTURE_METHOD
                    [cable_forces, id_exit_type] = id_cf_ipm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.ALTERNATE_IMPROVED_PUNCTURE_METHOD
                    [cable_forces, id_exit_type] = id_cf_aipm(A_eq, b_eq, fmin, fmax);    
                case ID_CF_SolverType.WARM_IMPROVED_PUNCTURE_METHOD
                    [cable_forces, id_exit_type,obj.active_set,obj.x_fixed] = id_cf_wipm(A_eq, b_eq, fmin, fmax, obj.active_set,obj.x_fixed);    
                otherwise
                    CASPR_log.Print('ID_CF_SolverType type is not defined',CASPRLogLevel.ERROR);
            end

            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                cable_forces = dynamics.ACTUATION_ACTIVE_INVALID;
                Q_opt = Inf;
            else
                Q_opt = norm(cable_forces);
            end

            obj.f_previous = cable_forces;
        end
    end
end
