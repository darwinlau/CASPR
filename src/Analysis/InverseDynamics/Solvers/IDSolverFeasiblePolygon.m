% An efficient inverse dynamics solver for CDPR systems with n+2 cables
% (where n is the number of DOFs). 
%
% Please cite the following paper when using this algorithm:
% M. Gouttefarde, J. Lamaury, C. Reichert and T. Bruckmann, "A Versatile
% Tension Distribution Algorithm for n-DOF Parallel Robots Driven by n+2
% Cables", IEEE Trans. Robot., vol. 31, no. 6, pp. 1444-1457, 2015.
%
% Author        : Jonathan EDEN and Jihong ZHU
% Created       : 2016
% Description   : This approach allows for three types of objective
% functions to be considered, to minimise the 1-norm of cable forces, to 
% minimise the 2-norm of cable forces, and to produce a wrench closest to 
% centroid of the wrench polytope.
classdef IDSolverFeasiblePolygon < IDSolverBase    
    properties (SetAccess = private)
        fp_solver_type
    end
    methods
        function id = IDSolverFeasiblePolygon(model,fp_solver_type)
            id@IDSolverBase(model);
            id.fp_solver_type = fp_solver_type;
        end
        
        function [cable_forces,Q_opt, id_exit_type] = resolveFunction(obj, dynamics)            
            % Ensure that the resolve function should be applied for this
            % class of problem
            assert(dynamics.numCables == dynamics.numDofs + 2,'Number of cables must be equal the number of degrees of freedom plus 2');
            
            % Form the linear EoM constraint
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A_eq, b_eq] = IDSolverBase.GetEoMConstraints(dynamics);  
            % Form the lower and upper bound force constraints
            fmin = dynamics.forcesMin;
            fmax = dynamics.forcesMax;

            switch (obj.fp_solver_type)
                case ID_FP_SolverType.NORM_1
                    [cable_forces, id_exit_type] = id_fp_1_norm(A_eq, b_eq, fmin, fmax);
                    Q_opt = norm(cable_forces,1);
                case ID_FP_SolverType.NORM_2
                    [cable_forces, id_exit_type] = id_fp_2_norm(A_eq, b_eq, fmin, fmax);
                    Q_opt = norm(cable_forces);
                case ID_FP_SolverType.CENTROID
                    [cable_forces, id_exit_type] = id_fp_centroid(A_eq, b_eq, fmin, fmax);
                    Q_opt = 0;
                otherwise
                    error('ID_FP_SolverType type is not defined');
            end
            
            if (id_exit_type ~= IDSolverExitType.NO_ERROR)
                cable_forces = dynamics.forcesInvalid;
                Q_opt = inf;
                %id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            end            
            
            obj.f_previous = cable_forces;
        end
    end
end

