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
            
            % Setup initial pose
            obj.model.update(q0, q0_dot, zeros(size(q0_dot)));
            q0_ddot = obj.model.q_ddot_dynamics;
            obj.model.update(q0, q0_dot, q0_ddot);
            
            obj.trajectory.q{1} = q0;
            obj.trajectory.q_dot{1} = q0_dot;
            obj.trajectory.q_ddot{1} = q0_ddot;
            
            for t = 2:length(obj.timeVector)
                fprintf('Simulation time : %f\n', obj.timeVector(t));
                [obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t}, obj.model] = ForwardDynamics.Compute(obj.model.q, obj.model.q_dot, cable_forces{t-1}, obj.timeVector(t)-obj.timeVector(t-1), obj.model);
                obj.interactionWrench{t} = obj.model.interactionWrench;
                obj.cableLengths{t} = obj.model.cableLengths;
                obj.cableLengthsDot{t} = obj.model.cableLengthsDot;
            end
        end        
    end
end