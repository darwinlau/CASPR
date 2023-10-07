% Basic Inverse Dynamics solver for infinite norm problems.
% This is a well-studied form of inverse dynamics solver for CDPRs.
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description   : Only a linear objective function and linear
% constraints can be used with this solver. There are multiple types of LP
% solver implementations that can be used with this solver.
classdef IDSolverMinInfNorm < IDSolverBase

    properties (SetAccess = private)
        lp_solver_type
        objective
        constraints = {}
        options
        is_OptiToolbox
    end
    methods
        % The constructor for the class.
        function id = IDSolverMinInfNorm(model, objective, lp_solver_type)
            id@IDSolverBase(model);
            id.objective = objective;
            id.lp_solver_type = lp_solver_type;
            id.options = [];
            % Test if OptiToolbox is installed
            if(isempty(strfind(path,'OptiToolbox'))&&((id.lp_solver_type==ID_LP_SolverType.OPTITOOLBOX_OOQP)||(id.lp_solver_type==ID_LP_SolverType.OPTITOOLBOX_LP_SOLVE)))
                CASPR_log.Print('OptiToolbox is not installed, switching to MATLAB solver',CASPRLogLevel.WARNING);
                id.is_OptiToolbox = 0;
            else
                id.is_OptiToolbox = 1;
            end
        end

        % The implementation for the resolveFunction
        function [actuation_forces, Q_opt, id_exit_type] = resolveFunction(obj, dynamics)
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + w_{ext} = -L_active^T f_active - L_passive^T f_passive (constraint)
            [A_eq, b_eq] = IDSolverBase.GetEoMConstraints(dynamics);
            % Form the lower and upper bound force constraints
            fmin = dynamics.actuationForcesMin;
            fmax = dynamics.actuationForcesMax;
            % Get objective function
            obj.objective.updateObjective(dynamics);

            A_ineq = [];
            b_ineq = [];
            for i = 1:length(obj.constraints)
                obj.constraints{i}.updateConstraint(dynamics);
                A_ineq = [A_ineq; obj.constraints{i}.A];
                b_ineq = [b_ineq; obj.constraints{i}.b];
            end

            % Modify the inf norm
            [n,m] = size(A_eq);
            f = [zeros(m,1);1];
            A_ineq = [A_ineq,zeros(size(A_ineq,1),1);obj.objective.A,-ones(m,1);-obj.objective.A,-ones(m,1)];
            b_ineq = [b_ineq;-obj.objective.b;obj.objective.b];
            A_eq = [A_eq,zeros(n,1)];
            fmin = [fmin;-Inf];
            fmax = [fmax;Inf];
            if(~isempty(obj.f_previous))
                f0 = [obj.f_previous; 0];
            else
                f0 = zeros(m+1,1);
            end

            switch (obj.lp_solver_type)
                case ID_LP_SolverType.MATLAB
                    if(isempty(obj.options))
                        obj.options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'dual-simplex');
                    end
                    [temp_cable_forces, id_exit_type] = id_lp_matlab(f, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.options);
                    actuation_forces = temp_cable_forces(1:m);
                case ID_LP_SolverType.OPTITOOLBOX_OOQP
                    if(obj.is_OptiToolbox)
                        if(isempty(obj.options))
                            obj.options = optiset('solver', 'OOQP', 'maxiter', 100);
                        end
                        [temp_cable_forces, id_exit_type] = id_lp_opti(f, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, f0,obj.options);
                        actuation_forces = temp_cable_forces(1:m);
                    else
                        if(isempty(obj.options))
                            obj.options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'interior-point');
                        end
                        [temp_cable_forces, id_exit_type] = id_lp_matlab(f, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.options);
                        actuation_forces = temp_cable_forces(1:m);
                    end
                case ID_LP_SolverType.OPTITOOLBOX_LP_SOLVE
                    if(obj.is_OptiToolbox)
                        if(isempty(obj.options))
                            obj.options = optiset('solver', 'LP_SOLVE', 'maxiter', 100,'display','off','warnings','none');
                        end
                        [temp_cable_forces, id_exit_type] = id_lp_opti(f, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, f0,obj.options);
                        actuation_forces = temp_cable_forces(1:m);
                    else
                        if(isempty(obj.options))
                            obj.options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'interior-point');
                        end
                        [temp_cable_forces, id_exit_type] = id_lp_matlab(f, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.options);
                        actuation_forces = temp_cable_forces(1:m);
                    end
                otherwise
                    CASPR_log.Print('ID_LP_SolverType type is not defined',CASPRLogLevel.ERROR);
            end

            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                actuation_forces = dynamics.ACTUATION_ACTIVE_INVALID;
                Q_opt = Inf;
            else
                Q_opt = max(actuation_forces);
            end

            obj.f_previous = actuation_forces;
        end

        % A function with which to add additional constraaints
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end
end
