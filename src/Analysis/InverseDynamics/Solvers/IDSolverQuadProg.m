% Basic Inverse Dynamics solver for problems in the Quadratic Program form
% This is a well-studied form of inverse dynamics solver for CDPRs.
%
% Author        : Darwin LAU
% Created       : 2015
% Description   : Only a quadratic objective function and linear
% constraints can be used with this solver. There are multiple types of QP
% solver implementations that can be used with this solver.
classdef IDSolverQuadProg < IDSolverBase

    properties (SetAccess = private)
        qp_solver_type
        objective
        constraints = {}
        options
        is_OptiToolbox
    end
    methods
        % The contructor for the class.
        function id = IDSolverQuadProg(model, objective, qp_solver_type)
            id@IDSolverBase(model);
            id.objective = objective;
            id.qp_solver_type = qp_solver_type;
            id.active_set = [];
            id.options = [];
            % Test if OptiToolbox is installed
            if(isempty(strfind(path,'OptiToolbox'))&&((id.qp_solver_type == ID_QP_SolverType.OPTITOOLBOX_IPOPT)||(id.qp_solver_type == ID_QP_SolverType.OPTITOOLBOX_OOQP)))
                CASPR_log.Print('OptiToolbox is not installed, switching to MATLAB solver',CASPRLogLevel.WARNING);
                id.is_OptiToolbox = 0;
            else
                id.is_OptiToolbox = 1;
            end
        end

        % The implementation of the resolve function.
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

            % Solves the QP ID different depending on the solver type
            switch (obj.qp_solver_type)
                % Basic version that uses MATLAB's solver
                case ID_QP_SolverType.MATLAB
                    if(isempty(obj.options))
                        % solve the potential naming issues in matlab
                        [~, d] = version;
                        % derive the publish year of the Matlab being used
                        year = str2double(d(length(d)-3:length(d)));
                        if year >= 2016
                            tol_string = 'StepTolerance';
                        else
                            tol_string = 'TolX';
                        end
                        obj.options = optimoptions('quadprog', tol_string, 1e-17, 'Display', 'off', 'MaxIter', 100);
                    end
                    [actuation_forces, id_exit_type] = id_qp_matlab(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);                    
                % Basic version that uses MATLAB's solver
                case ID_QP_SolverType.MATLAB_INTERIOR_POINT
                    if(isempty(obj.options))
                        obj.options = optimoptions('quadprog','Algorithm','interior-point-convex', 'ConstraintTolerance', 1e-1, 'Display', 'off', 'MaxIter', 100);
                    end
                    [actuation_forces, id_exit_type] = id_qp_matlab(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);                    
                % Uses MATLAB solver with a warm start strategy on the
                % active set
                case ID_QP_SolverType.MATLAB_ACTIVE_SET_WARM_START
                    if(isempty(obj.options))
                        obj.options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
                    end
                    [actuation_forces, id_exit_type,obj.active_set] = id_qp_matlab_active_set_warm_start(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.active_set,obj.options);
                % Uses the IPOPT algorithm from OptiToolbox
                case ID_QP_SolverType.OPTITOOLBOX_IPOPT
                    if(obj.is_OptiToolbox)
                        if(isempty(obj.options))
                            obj.options = optiset('solver', 'IPOPT', 'maxiter', 100);
                        end
                        [actuation_forces, id_exit_type] = id_qp_opti(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                    else
                        if(isempty(obj.options))
                            obj.options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
                        end
                        [actuation_forces, id_exit_type] = id_qp_matlab(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                    end
                % Uses the OOQP algorithm from the Optitoolbox
                case ID_QP_SolverType.OPTITOOLBOX_OOQP
                    if(obj.is_OptiToolbox)
                        if(isempty(obj.options))
                            obj.options = optiset('solver', 'OOQP', 'maxiter', 100);
                        end
                        [actuation_forces, id_exit_type] = id_qp_opti(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                    else
                        if(isempty(obj.options))
                            obj.options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
                        end
                        [actuation_forces, id_exit_type] = id_qp_matlab(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                    end
                otherwise
                    CASPR_log.Print('ID_QP_SolverType type is not defined',CASPRLogLevel.ERROR);
            end

            % If there is an error, cable forces will take on the invalid
            % value and Q_opt is infinity
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                actuation_forces = dynamics.ACTUATION_ACTIVE_INVALID;
                Q_opt = inf;
            % Otherwise valid exit, compute Q_opt using the objective
            else
                Q_opt = obj.objective.evaluateFunction(actuation_forces);
            end
            % Set f_previous, may be useful for some algorithms
            obj.f_previous = actuation_forces;
        end

        % Helps to add an additional constraint to the QP problem
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end
end
