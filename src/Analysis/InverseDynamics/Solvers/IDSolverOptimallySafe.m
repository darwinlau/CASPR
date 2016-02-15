% An efficient inverse dynamics solver for CDPR systems with the objective
% to achieve optimally safe cable forces (avoiding cable force limits)
%
% Please cite the following paper when using this algorithm:
% P. H. Borgstrom, B. L. Jordan, G. S. Sukhatme, M. A. Batalin, and W. J.
% Kaiser, "Rapid Computation of Optimally Safe Tension Distributions for
% Parallel Cable-Driven Robots", IEEE Trans. Robot., vol. 25, no. 6, pp.
% 1271-1281.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : 
classdef IDSolverOptimallySafe < IDSolverBase    
    properties (SetAccess = private)
        os_solver_type
        x_prev
        alpha
    end
    methods
        function q = IDSolverOptimallySafe(os_solver_type)
            q.os_solver_type = os_solver_type;
            q.x_prev = [];
            q.alpha = 1;
        end
        
        function [Q_opt, id_exit_type] = resolveFunction(obj, dynamics)            
            % ASK DARWIN REGARDING IF THIS SHOULD BE SEPERATE OR PART OF
            % LINPROG. If so add alpha as a variable
            
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A_eq, b_eq] = IDSolverFunction.GetEoMConstraints(dynamics);  
            % Form the lower and upper bound force constraints
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;

            % Ensure that the resolve function should be applied for this
            % class of problem
            assert(sum(fmin-fmin(1)*ones(size(fmin))) + sum(fmax-fmax(1)*ones(size(fmax))) == 0,'Minimum and maximum cable forces should be the same for all cables.');
            
            switch (obj.os_solver_type)
                case ID_OS_SolverType.LP
                    [dynamics.cableForces, id_exit_type] = id_os_matlab(A_eq, b_eq, fmin, fmax,obj.alpha);
                    Q_opt = norm(dynamics.cableForces,1);
                case ID_OS_SolverType.EFFICIENT_LP
                    [dynamics.cableForces, id_exit_type,obj.x_prev,obj.active_set] = id_os_efficient(A_eq, b_eq, fmin, fmax,obj.alpha,obj.x_prev,obj.active_set);
                    Q_opt = norm(dynamics.cableForces);
                otherwise
                    error('ID_OS_SolverType type is not defined');
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

