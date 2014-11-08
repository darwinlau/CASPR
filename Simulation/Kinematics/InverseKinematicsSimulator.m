classdef InverseKinematicsSimulator < MotionSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)
    
    properties (SetAccess = protected)                       
        trajectory          % Trajectory object for inverse problems only (input)
    end
    
    methods
        function ik = InverseKinematicsSimulator()
        end
        
        function run(obj, trajectory, skConstructor)
            obj.trajectory = trajectory;
            
            obj.timeVector = obj.trajectory.timeVector;
            
            % Runs the simulation over the specified trajectory
            obj.states = CellOperations.CreateCellArray(skConstructor, [1 length(obj.trajectory.timeVector)]);
            
            for t = 1:length(obj.trajectory.timeVector)
                fprintf('Time : %f\n', obj.trajectory.timeVector(t));
                obj.states{t}.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t});
            end
        end
    end
end

