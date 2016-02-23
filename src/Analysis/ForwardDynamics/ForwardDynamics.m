classdef ForwardDynamics < handle
    %FORWARDDYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        function [q, q_dot, q_ddot, dynamics] = Compute(qp, qp_d, cable_forces, w_ext, dt, model)
            n_vars = model.numDofVars;
            n_dofs = model.numDofs;
            % The procedure to perform the forward dynamics
            % Initialise the starting point for the ODE
            y0 = [qp; qp_d];
            % Run the ODE function
            [~, y_out] = ode113(@(~,y) ForwardDynamics.eom(0, y, model, cable_forces, w_ext), [0 dt], y0);
            % The output of the ODE is the solution and y0 for the next iteration
            s_end = size(y_out,1);
            % Store the q and q_dot values
            q = y_out(s_end, 1:n_vars)';
            q_dot = y_out(s_end, n_vars+1:length(y0))';
            % Update the model with q, q_dot and f so that q_ddot can be determined
            model.update(q, q_dot, zeros(n_dofs, 1), w_ext);
            model.cableForces = cable_forces;
            % Determine this using the dynamics of the system and the
            % cable forces
            q_ddot = model.q_ddot_dynamics;
            dynamics = model;
        end        
    end
    
    methods (Static, Access = private)        
        function y_dot = eom(~, y, model, f, w_ext)
            n_vars = model.numDofVars;
            n_dofs = model.numDofs;
            q = y(1:n_vars);
            q_dot = y(n_vars+1:length(y));

            y_dot = zeros(size(y));

            model.update(q, q_dot, zeros(n_dofs, 1), w_ext);
            model.cableForces = f;

            y_dot(1:n_vars) = model.q_deriv;
            y_dot(n_vars+1:length(y)) = model.q_ddot_dynamics;
        end
    end
end