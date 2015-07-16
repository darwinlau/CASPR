classdef InverseKinematicsSimulator < MotionSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)
    
    properties (SetAccess = protected)       
    end
    
    methods
        function ik = InverseKinematicsSimulator(model)
            ik@MotionSimulator(model);
        end
        
        function run(obj, trajectory)
            obj.trajectory = trajectory;
            
            obj.timeVector = obj.trajectory.timeVector;
            
            % Runs the simulation over the specified trajectory
            obj.lengths = cell(1, length(obj.trajectory.timeVector));
            obj.lengths_dot = cell(1, length(obj.trajectory.timeVector));
            
            for t = 1:length(obj.trajectory.timeVector)
                fprintf('Time : %f\n', obj.trajectory.timeVector(t));
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t});
                obj.lengths{t} = obj.model.cableLengths;
                obj.lengths_dot{t} = obj.model.cableLengthsDot;
            end
        end
    end
end

