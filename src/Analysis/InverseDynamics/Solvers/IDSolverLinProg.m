classdef IDSolverLinProg < IDSolverFunction
    %IDFUNCTIONQP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        objective
        lp_solver_type
    end
    methods
        function q = IDSolverLinProg(objective, lp_solver_type)
            q.objective = objective;
            q.lp_solver_type = lp_solver_type;
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
                        
            switch (obj.lp_solver_type)
                case ID_LP_SolverType.MATLAB
                    [dynamics.cableForces, id_exit_type, comp_time] = id_lp_optitoolbox_clp(obj.objective.b, [], [], A_eq, b_eq, fmin, fmax, obj.f_previous);
                case ID_LP_SolverType.OPTITOOLBOX_CLP
                    [dynamics.cableForces, id_exit_type, comp_time] = id_lp_optitoolbox_clp(obj.objective.b, [], [], A_eq, b_eq, fmin, fmax, obj.f_previous);
                otherwise
                    error('ID_LP_SolverType type is not defined');
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

