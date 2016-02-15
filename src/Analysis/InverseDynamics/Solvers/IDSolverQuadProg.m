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
    end
    methods
        function q = IDSolverQuadProg(objective, qp_solver_type)
            q.objective = objective;
            q.qp_solver_type = qp_solver_type;
            q.active_set = [];
            q.options = [];
        end
        
        function [Q_opt, id_exit_type] = resolveFunction(obj, dynamics)            
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A_eq, b_eq] = IDSolverBase.GetEoMConstraints(dynamics);  
            % Form the lower and upper bound force constraints
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;
            % Get objective function
            obj.objective.updateObjective(dynamics);

            A_ineq = [];
            b_ineq = [];
            for i = 1:length(obj.constraints)
                obj.constraints{i}.updateConstraint(dynamics);
                A_ineq = [A_ineq; obj.constraints{i}.A];
                b_ineq = [b_ineq; obj.constraints{i}.b];                
            end

            switch (obj.qp_solver_type)
                case ID_QP_SolverType.MATLAB
                    if(isempty(obj.options))
                        obj.options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
                    end 
                    [dynamics.cableForces, id_exit_type] = id_qp_matlab(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                case ID_QP_SolverType.MATLAB_ACTIVE_SET_WARM_START
                    if(isempty(obj.options))
                        obj.options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
%                         obj.options = optiset('solver', 'OOQP', 'maxiter', 100);
                    end 
                    [dynamics.cableForces, id_exit_type,obj.active_set] = id_qp_matlab_active_set_warm_start(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.active_set,obj.options);
                case ID_QP_SolverType.OPTITOOLBOX_IPOPT
                    [dynamics.cableForces, id_exit_type] = id_qp_optitoolbox_ipopt(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous);
                case ID_QP_SolverType.OPTITOOLBOX_OOQP
                    if(isempty(obj.options))
                        obj.options = optiset('solver', 'OOQP', 'maxiter', 100);
                    end
                    [dynamics.cableForces, id_exit_type] = id_qp_optitoolbox_ooqp(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                otherwise
                    error('ID_QP_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Q_opt = inf;
            else
                Q_opt = obj.objective.evaluateFunction(dynamics.cableForces);
            end            
            
            obj.f_previous = dynamics.cableForces;
        end
        
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end
    
end

