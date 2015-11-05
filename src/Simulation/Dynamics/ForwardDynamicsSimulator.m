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
            
            n_dof = length(q0);
            
            % Setup initial pose
            obj.model.update(q0, q0_dot, zeros(size(q0)));
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
                obj.trajectory.q{t} = y_out(s_end, 1:n_dof)';
                obj.trajectory.q_dot{t} = y_out(s_end, n_dof+1:2*n_dof)';
                % Update the model with q, q_dot and f so that q_ddot can be determined
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, zeros(n_dof, 1));
                obj.model.cableForces = cable_forces{t-1};
                % Determine this using the dynamics of the system and the
                % cable forces
                obj.trajectory.q_ddot{t} = obj.model.q_ddot_dynamics;
                obj.interactionWrench{t} = obj.model.interactionWrench;
            end
        end        
    end
    
%     methods (Static)
%         function [nKinematics, nDynamics] = ForwardDynamics(cKinematics, f, t, t_span)
%             n_dof = cKinematics.n_dof;
%             y0 = [cKinematics.q; cKinematics.q_dot];
%             
%             [~, out] = ode45(@(time,y) eom(time,y, cKinematics.BodyKinematics.BodiesProp, cKinematics.CableKinematics.CablesProp, f), [t t+t_span], y0);
%             
%             s = size(out);
%             q = out(s(1), 1:n_dof)';
%             q_dot = out(s(1), n_dof+1:2*n_dof)';
%             
%             nKinematics = SystemKinematics(cKinematics.BodyKinematics.BodiesProp, cKinematics.CableKinematics.CablesProp);
%             nKinematics.SetState(q, q_dot, zeros(n_dof, 1));
%             nDynamics = SystemDynamics(nKinematics);
%             nDynamics.CableForces = f;          
%             q_ddot = nDynamics.q_ddot_dynamics;
%             nKinematics.SetState(q, q_dot, q_ddot);
%         end
%     end
end
    
function y_dot = eom(~, y, model, f)
    n_dof = length(y)/2;
    q = y(1:n_dof);
    q_dot = y(n_dof+1:2*n_dof);
    
    y_dot = zeros(2*n_dof, 1);
    
    model.update(q, q_dot, zeros(n_dof, 1));
    model.cableForces = f;
        
    y_dot(1:n_dof) = q_dot;
    y_dot(n_dof+1:2*n_dof) = model.q_ddot_dynamics;
end

