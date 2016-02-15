classdef ForwardDynamicsSimulator < DynamicsSimulator
    %FORWARDDYNAMICSSIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties        
    end
    
    methods
        function id = ForwardDynamicsSimulator(model)
            id@DynamicsSimulator(model);
        end
        
        
        function run(obj, cable_forces, time_vector, q0, q0_dot)
            obj.timeVector = time_vector;
            obj.cableForces = cable_forces;
            
            obj.trajectory = JointTrajectory;
            obj.trajectory.timeVector = obj.timeVector;
            obj.trajectory.q = cell(1, length(obj.timeVector));
            obj.trajectory.q_dot = cell(1, length(obj.timeVector));
            obj.trajectory.q_ddot = cell(1, length(obj.timeVector));
            
            n_vars = obj.model.numDofVars;
            n_dofs = obj.model.numDofs;
            
            % Setup initial pose
            obj.model.update(q0, q0_dot, zeros(size(q0_dot)));
            q0_ddot = obj.model.q_ddot_dynamics;
            obj.model.update(q0, q0_dot, q0_ddot);
            
            obj.trajectory.q{1} = q0;
            obj.trajectory.q_dot{1} = q0_dot;
            obj.trajectory.q_ddot{1} = q0_ddot;
            
            for t = 2:length(obj.timeVector)
                fprintf('Simulation time : %f\n', obj.timeVector(t));
                
                % The procedure to perform the forward dynamics
                % Initialise the starting point for the ODE
                y0 = [obj.model.q; obj.model.q_dot];
                % Run the ODE function
                [~, y_out] = ode113(@(time,y) eom(time, y, obj.model, cable_forces{t-1}), [obj.timeVector(t-1) obj.timeVector(t)], y0);
                % The output of the ODE is the solution and y0 for the next iteration
                s_end = size(y_out,1);
                % Store the q and q_dot values
                obj.trajectory.q{t} = y_out(s_end, 1:n_vars)';
                obj.trajectory.q_dot{t} = y_out(s_end, n_vars+1:length(y0))';
                % Update the model with q, q_dot and f so that q_ddot can be determined
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, zeros(n_dofs, 1));
                obj.model.cableForces = cable_forces{t-1};
                % Determine this using the dynamics of the system and the
                % cable forces
                obj.trajectory.q_ddot{t} = obj.model.q_ddot_dynamics;
                obj.interactionWrench{t} = obj.model.interactionWrench;
            end
        end        
    end
end
    
function y_dot = eom(~, y, model, f)
    n_vars = model.numDofVars;
    n_dofs = model.numDofs;
    q = y(1:n_vars);
    q_dot = y(n_vars+1:length(y));
    
    y_dot = zeros(size(y));
    
    model.update(q, q_dot, zeros(n_dofs, 1));
    model.cableForces = f;
        
    y_dot(1:n_vars) = model.q_deriv;
    y_dot(n_vars+1:length(y)) = model.q_ddot_dynamics;
end

