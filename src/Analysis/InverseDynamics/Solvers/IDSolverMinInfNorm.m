% Basic Inverse Dynamics solver for problems infinite norm problems.
% This is a well-studied form of inverse dynamics solver for CDPRs.
%
% Author        : Darwin LAU
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
    end
    methods
        function id = IDSolverMinInfNorm(model, objective, lp_solver_type)
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
            
            % Modify the info norm
            [n,m] = size(A_eq);
            f = [zeros(m,1);1];
            W = diag(obj.objective.b);
            A_ineq = [A_ineq,zeros(size(A_ineq,1),1);W,-ones(m,1)];
            b_ineq = [b_ineq;zeros(m,1)];
            A_eq = [A_eq,zeros(n,1)];
            fmin = [fmin;-Inf];
            fmax = [fmax;Inf];
            if(~isempty(obj.f_previous))
                f0 = [obj.f_previous;0];
            else
                f0 = zeros(m+1,1);
            end
            
            switch (obj.lp_solver_type)
                case ID_LP_SolverType.MATLAB
                    if(isempty(obj.options))
                        obj.options = optimoptions('linprog', 'Display', 'off', 'Algorithm', 'interior-point');
                    end
                    [temp_cable_forces, id_exit_type] = id_lp_matlab(f, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, f0,obj.options);
                    cable_forces = temp_cable_forces(1:m);
                case ID_LP_SolverType.OPTITOOLBOX_CLP
                    [temp_cable_forces, id_exit_type] = id_lp_optitoolbox_clp(f, A_ineq, b_ineq, A_eq, b_eq, fmin, fmax, f0);
                    cable_forces = temp_cable_forces(1:m);
                otherwise
                    error('ID_LP_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                cable_forces = dynamics.cableDynamics.forcesInvalid;
                Q_opt = inf;
                %id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            else
                Q_opt = max(cable_forces);
            end            
            
            obj.f_previous = cable_forces;
        end
        
        function addConstraint(obj, linConstraint)
            obj.constraints{length(obj.constraints)+1} = linConstraint;
        end
    end
end

