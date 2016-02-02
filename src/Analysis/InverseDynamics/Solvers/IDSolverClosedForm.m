classdef IDSolverClosedForm < IDSolverFunction
    %IDFUNCTIONQP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        cf_solver_type
        options
    end
    methods
        function q = IDSolverClosedForm(cf_solver_type)
            q.cf_solver_type = cf_solver_type;
        end
        
        function [Q_opt, id_exit_type] = resolveFunction(obj, dynamics)            
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A_eq, b_eq] = IDSolverFunction.GetEoMConstraints(dynamics);  
            % Form the lower and upper bound force constraints
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;

            switch (obj.cf_solver_type)
                case ID_CF_SolverType.CLOSED_FORM
                    [dynamics.cableForces, id_exit_type] = id_cf_cfm(A_eq, b_eq, fmin, fmax);
                case ID_CF_SolverType.ICFM
                    [dynamics.cableForces, id_exit_type] = id_cf_icfm(A_eq, b_eq, fmin, fmax);
                otherwise
                    error('ID_CF_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Q_opt = inf;
                %id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            else
                Q_opt = norm(dynamics.cableForces);
            end            
            
            obj.f_previous = dynamics.cableForces;
        end
        
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end
    
end

