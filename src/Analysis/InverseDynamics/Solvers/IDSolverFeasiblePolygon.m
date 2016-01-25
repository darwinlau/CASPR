classdef IDSolverFeasiblePolygon < IDSolverFunction
    %IDFUNCTIONQP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        fp_solver_type
    end
    methods
        function q = IDSolverFeasiblePolygon(fp_solver_type)
            q.fp_solver_type = fp_solver_type;
        end
        
        function [Q_opt, id_exit_type] = resolveFunction(obj, dynamics)            
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A_eq, b_eq] = IDSolverFunction.GetEoMConstraints(dynamics);  
            % Form the lower and upper bound force constraints
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;

            switch (obj.fp_solver_type)
                case ID_FP_SolverType.NORM_1
                    [dynamics.cableForces, id_exit_type] = id_fp_1_norm(A_eq, b_eq, fmin, fmax);
                    Q_opt = norm(dynamics.cableForces,1);
                case ID_FP_SolverType.NORM_2
                    [dynamics.cableForces, id_exit_type] = id_fp_2_norm(A_eq, b_eq, fmin, fmax);
                    Q_opt = norm(dynamics.cableForces);
                case ID_FP_SolverType.CENTROID
                    [dynamics.cableForces, id_exit_type] = id_fp_centroid(A_eq, b_eq, fmin, fmax);
                    Q_opt = 0;
                otherwise
                    error('ID_FP_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Q_opt = inf;
                %id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            end            
            
            obj.f_previous = dynamics.cableForces;
        end
        
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end
    
end

