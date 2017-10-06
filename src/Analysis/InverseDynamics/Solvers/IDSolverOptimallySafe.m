% An efficient inverse dynamics solver for CDPR systems with the objective
% to achieve optimally safe cable forces (avoiding cable force limits)
%
% Please cite the following paper when using this algorithm:
% P. H. Borgstrom, B. L. Jordan, G. S. Sukhatme, M. A. Batalin, and W. J.
% Kaiser, "Rapid Computation of Optimally Safe Tension Distributions for
% Parallel Cable-Driven Robots", IEEE Trans. Robot., vol. 25, no. 6, pp.
% 1271-1281, 2011.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef IDSolverOptimallySafe < IDSolverBase
    properties (SetAccess = private)
        os_solver_type
        x_prev
        alpha
        options
    end
    methods
        % The constructor for this class.
        function id = IDSolverOptimallySafe(model,alpha,os_solver_type)
            id@IDSolverBase(model);
            id.os_solver_type = os_solver_type;
            id.x_prev = [];
            id.options = [];
            id.alpha = alpha;
            CASPR_log.Assert((length(alpha) == 1) && (alpha >= 0),'alpha must be positive ');
        end

        % The implementation of the abstract resolveFunction
        function [cable_forces,Q_opt, id_exit_type] = resolveFunction(obj, dynamics)
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + w_{ext} = -L_active^T f_active - L_passive^T f_passive (constraint)
            [A_eq, b_eq] = IDSolverBase.GetEoMConstraints(dynamics);
            % Form the lower and upper bound force constraints
            fmin = dynamics.actuationForcesMin;
            fmax = dynamics.actuationForcesMax;

            % Ensure that the resolve function should be applied for this
            % class of problem
            CASPR_log.Assert(sum(fmin-fmin(1)*ones(size(fmin))) + sum(fmax-fmax(1)*ones(size(fmax))) == 0,'Minimum and maximum cable forces should be the same for all cables.');
            CASPR_log.Assert((obj.alpha >= 0) && ~isinf(obj.alpha),'alpha must be non-negative and finite.');

            switch (obj.os_solver_type)
                case ID_OS_SolverType.LP
                    if(isempty(obj.options))
                        obj.options = optimoptions('linprog', 'Algorithm','dual-simplex','Display', 'off', 'MaxIter', 100);
                    end
                    [cable_forces, id_exit_type] = id_os_matlab(A_eq, b_eq, fmin, fmax, obj.alpha,obj.options);
                    Q_opt = norm(cable_forces,1);
                case ID_OS_SolverType.EFFICIENT_LP
                    if(isempty(obj.options))
                        obj.options = optimoptions('linprog', 'Algorithm','dual-simplex','Display', 'off', 'MaxIter', 100);
                    end
                    [cable_forces, id_exit_type,obj.x_prev,obj.active_set] = id_os_efficient(A_eq, b_eq, fmin, fmax, obj.alpha, obj.x_prev, obj.active_set,obj.options);
                    Q_opt = norm(cable_forces);
                otherwise
                    CASPR_log.Print('ID_OS_SolverType type is not defined',CASPRLogLevel.ERROR);
            end

            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                cable_forces = dynamics.cableModel.FORCES_ACTIVE_INVALID;
                Q_opt = inf;
            end

            obj.f_previous = cable_forces;
        end

        % A function to add constraints.
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end

end
