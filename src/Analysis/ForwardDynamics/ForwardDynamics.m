% Performs the forward dynamics for cable-driven robots
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%   The foward dynamics is performed using a simple scheme of
%   double-integration of the joint acceleration obtained from the
%   equations of motion. The solver type of the FD can be specified.
%   Currently, the solver types are limited to the MATLAB ODE solvers.
classdef ForwardDynamics < handle
    properties
        solverType          % The type of forward dynamics solver
    end

    methods
        % A constructor for forward dynamics objects
        function fd = ForwardDynamics(solver_type)
            fd.solverType = solver_type;
        end

        % Compute the forward dynamics given a set of desired joint space
        % positions and cable forces.
        function [q, q_dot, q_ddot, dynamics] = compute(obj, qp, qp_d, cable_forces_active, cable_indices_active, w_ext, dt, model)
            n_vars = model.numDofVars;
            n_dofs = model.numDofs;
            % The procedure to perform the forward dynamics
            % Initialise the starting point for the ODE
            y0 = [qp; qp_d];
            % Run the ODE function
            switch (obj.solverType)
                case FDSolverType.ODE45
                    solverFn = @(f,t,y0) ode45(f,t,y0);
                case FDSolverType.ODE23
                    solverFn = @(f,t,y0) ode23(f,t,y0);
                case FDSolverType.ODE113
                    solverFn = @(f,t,y0) ode113(f,t,y0);
                case FDSolverType.ODE15S
                    solverFn = @(f,t,y0) ode15s(f,t,y0);
                case FDSolverType.ODE23S
                    solverFn = @(f,t,y0) ode23s(f,t,y0);
                case FDSolverType.ODE23T
                    solverFn = @(f,t,y0) ode23t(f,t,y0);
                case FDSolverType.ODE23TB
                    solverFn = @(f,t,y0) ode23tb(f,t,y0);
            end
            [~, y_out] = solverFn(@(~,y) ForwardDynamics.eom(0, y, model, cable_forces_active, cable_indices_active, w_ext), [0 dt], y0);

            %[~, y_out] = ode113(@(~,y) ForwardDynamics.eom(0, y, model, cable_forces, w_ext), [0 dt], y0);
            % The output of the ODE is the solution and y0 for the next iteration
            s_end = size(y_out,1);
            % Store the q and q_dot values
            q = y_out(s_end, 1:n_vars)';
            q_dot = y_out(s_end, n_vars+1:length(y0))';
            % Update the model with q, q_dot and f so that q_ddot can be determined
            model.update(q, q_dot, zeros(n_dofs, 1), w_ext);
            model.cableForces = cable_forces_active;
            % Determine this using the dynamics of the system and the
            % cable forces
            q_ddot = model.q_ddot_dynamics;
            dynamics = model;
        end
    end

    methods (Static, Access = private)
        % The equation of motion for integration purposes.
        function y_dot = eom(~, y, model, f_active, f_indices_active, w_ext)
            n_vars = model.numDofVars;
            n_dofs = model.numDofs;
            q = y(1:n_vars);
            q_dot = y(n_vars+1:length(y));

            y_dot = zeros(size(y));

            model.update(q, q_dot, zeros(n_dofs, 1), w_ext);
            assert(isequal(model.cableModel.cableIndicesActive, f_indices_active), 'The cable forces that should be active do not match that of the input');
            model.cableForces = f_active;

            y_dot(1:n_vars) = model.q_deriv;
            y_dot(n_vars+1:length(y)) = model.q_ddot_dynamics;
        end
    end
end
