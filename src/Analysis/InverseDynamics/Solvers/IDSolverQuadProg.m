classdef IDSolverQuadProg < IDSolverFunction
    %IDFUNCTIONQP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        objective
        qp_solver_type
    end
    methods
        function q = IDSolverQuadProg(objective, qp_solver_type)
            q.objective = objective;
            q.qp_solver_type = qp_solver_type;
        end
        
        function [Q_opt, id_exit_type, comp_time] = resolve(obj, dynamics)            
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A_eq, b_eq] = IDSolverFunction.GetEoMConstraints(dynamics);  
            % Form the lower and upper bound force constraints
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;
            % Get objective function
            obj.objective.updateParameters(dynamics);
            
%             if isempty(obj.f_previous)
%                 obj.f_previous = zeros(dynamics.numCables, 1);
%             end
            
            switch (obj.qp_solver_type)
                case ID_QP_SolverType.MATLAB
                    [dynamics.cableForces, id_exit_type, comp_time] = id_qp_matlab(obj.objective.A, obj.objective.b, [], [], A_eq, b_eq, fmin, fmax, obj.f_previous);
                case ID_QP_SolverType.OPTITOOLBOX_IPOPT
                    [dynamics.cableForces, id_exit_type, comp_time] = id_qp_optitoolbox_ipopt(obj.objective.A, obj.objective.b, [], [], A_eq, b_eq, fmin, fmax, obj.f_previous);
                case ID_QP_SolverType.OPTITOOLBOX_OOQP
                    [dynamics.cableForces, id_exit_type, comp_time] = id_qp_optitoolbox_ooqp(obj.objective.A, obj.objective.b, [], [], A_eq, b_eq, fmin, fmax, obj.f_previous);
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
    end
    
end

