% Basic Inverse Dynamics solver for problems in the Linear Program form
% This is a well-studied form of inverse dynamics solver for CDPRs.
%
% Author        : Darwin LAU
% Created       : 2015
% Description   : Only a linear objective function and linear 
% constraints can be used with this solver. There are multiple types of LP
% solver implementations that can be used with this solver.
classdef IDSolverLinProg < IDSolverBase
    
    properties (SetAccess = private)
        lp_solver_type
        objective
        constraints = {}
        options
    end
    methods
        function id = IDSolverLinProg(model,objective, lp_solver_type)
            id@IDSolverBase(model);
            id.objective = objective;
            id.lp_solver_type = lp_solver_type;
            id.options = [];
        end
        
        function [cable_forces,Q_opt, id_exit_type] = resolveFunction(obj, dynamics)            
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
            
            switch (obj.lp_solver_type)
                case ID_LP_SolverType.MATLAB
                    if(isempty(obj.options))
                        obj.options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'interior-point');
                    end
                    [cable_forces, id_exit_type] = id_lp_matlab(obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                case ID_LP_SolverType.OPTITOOLBOX_OOQP
                    if(isempty(obj.options))
                        obj.options = optiset('solver', 'OOQP', 'maxiter', 100);
                    end 
                    [cable_forces, id_exit_type] = id_lp_opti(obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                case ID_LP_SolverType.OPTITOOLBOX_LP_SOLVE
                    if(isempty(obj.options))
                        obj.options = optiset('solver', 'LP_SOLVE', 'maxiter', 100,'display','off','warnings','none');
                    end
                    [cable_forces, id_exit_type] = id_lp_opti(obj.objective.b, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, obj.f_previous,obj.options);
                otherwise
                    error('ID_LP_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                cable_forces = dynamics.cableDynamics.forcesInvalid;
                Q_opt = inf;
                %id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            else
                Q_opt = obj.objective.evaluateFunction(cable_forces);
            end            
            
            obj.f_previous = cable_forces;
        end
        
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end
end

