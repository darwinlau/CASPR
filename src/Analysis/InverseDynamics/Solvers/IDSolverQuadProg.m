classdef IDSolverQuadProg < IDSolverFunction
    %IDFUNCTIONQP Summary of this class goes here
    %   Detailed explanation goes here
    
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
            [A_eq, b_eq] = IDSolverFunction.GetEoMConstraints(dynamics);  
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
                case ID_QP_SolverType.MATLAB_EFFICIENT
                    if(isempty(obj.options))
                        obj.options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
                    end 
                    [dynamics.cableForces, id_exit_type,obj.active_set] = id_qp_matlab_efficient(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.active_set,obj.options);
                case ID_QP_SolverType.OPTITOOLBOX_IPOPT
                    [dynamics.cableForces, id_exit_type] = id_qp_optitoolbox_ipopt(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous);
                case ID_QP_SolverType.OPTITOOLBOX_OOQP
                    [dynamics.cableForces, id_exit_type] = id_qp_optitoolbox_ooqp(obj.objective.A, obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous);
                otherwise
                    error('ID_QP_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Q_opt = inf;
                %id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
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

