classdef ForwardDynamicsSimulator < DynamicsSimulator
    %FORWARDDYNAMICSSIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties        
    end
    
    methods
        function id = ForwardDynamicsSimulator()
        end
        
        
        function Run(obj, cableforces, time_step, total_time, q0, q0_dot)
            obj.TimeVector = 0:time_step:total_time;            
            
            % Runs the simulation over the specified trajectory
            obj.Kinematics = cell(1, length(obj.TimeVector)); %SystemKinematics.InitialiseCellArray(length(obj.Trajectory.Time), obj.BodiesProp, obj.CablesProp);
            obj.Dynamics = cell(1, length(obj.TimeVector));
            
            % Setup initial pose
            obj.Kinematics{1} = SystemKinematics(obj.BodiesProp, obj.CablesProp);
            
            obj.Kinematics{1}.SetState(q0, q0_dot, zeros(length(q0),1));
            obj.Dynamics{1} = SystemDynamics(obj.Kinematics{1});
            obj.Dynamics{1}.CableForces = cableforces{1};
            q0_ddot = obj.Dynamics{1}.q_ddot_dynamics;
            obj.Kinematics{1}.SetState(q0, q0_dot, q0_ddot);
            
            for t = 2:length(obj.TimeVector)
                fprintf('Simulation time : %f\n', obj.TimeVector(t));
                [obj.Kinematics{t}, obj.Dynamics{t}] = ForwardDynamicsSimulator.ForwardDynamics(obj.Kinematics{t-1}, cableforces{t-1}, obj.TimeVector(t-1), time_step);
            end
        end        
    end
    
    methods (Static)
        function [nKinematics, nDynamics] = ForwardDynamics(cKinematics, f, t, t_span)
            n_dof = cKinematics.n_dof;
            y0 = [cKinematics.q; cKinematics.q_dot];
            
            [~, out] = ode45(@(time,y) eom(time,y, cKinematics.BodyKinematics.BodiesProp, cKinematics.CableKinematics.CablesProp, f), [t t+t_span], y0);
            
            s = size(out);
            q = out(s(1), 1:n_dof)';
            q_dot = out(s(1), n_dof+1:2*n_dof)';
            
            nKinematics = SystemKinematics(cKinematics.BodyKinematics.BodiesProp, cKinematics.CableKinematics.CablesProp);
            nKinematics.SetState(q, q_dot, zeros(n_dof, 1));
            nDynamics = SystemDynamics(nKinematics);
            nDynamics.CableForces = f;          
            q_ddot = nDynamics.q_ddot_dynamics;
            nKinematics.SetState(q, q_dot, q_ddot);
            q
        end
    end
end
    
function y_dot = eom(t, y, bp, cp, f)
    n_dof = length(y)/2;
    q = y(1:n_dof);
    q_dot = y(n_dof+1:2*n_dof);
    
    y_dot = zeros(2*n_dof, 1);
    
    kin = SystemKinematics(bp, cp);
    kin.SetState(q, q_dot, zeros(n_dof, 1));
    dyn = SystemDynamics(kin);
    dyn.CableForces = f;
    
    %q_ddot = dyn.M\(-dyn.J'*f - dyn.C - dyn.G);
    
    y_dot(1:n_dof) = q_dot;
    y_dot(n_dof+1:2*n_dof) = dyn.q_ddot_dynamics;
    
    %y_dot
end

