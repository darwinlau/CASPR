classdef InverseKinematicsSimulator < MotionSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)
    
    properties (SetAccess = protected)          
        trajectory      	% Trajectory object for inverse problems only (input)
        lengths             % cell array of cable lengths (output)
        lengths_dot         % cell array of cable lengths dot (output)
    end
    
    methods
        function ik = InverseKinematicsSimulator(kinObj)
            ik@MotionSimulator(kinObj);
        end
        
        function run(obj, trajectory)
            obj.trajectory = trajectory;
            
            obj.timeVector = obj.trajectory.timeVector;
            
            % Runs the simulation over the specified trajectory
            obj.lengths = cell(1, length(obj.trajectory.timeVector));
            obj.lengths_dot = cell(1, length(obj.trajectory.timeVector));
            
            for t = 1:length(obj.trajectory.timeVector)
                fprintf('Time : %f\n', obj.trajectory.timeVector(t));
                obj.kinematicsObj.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t});
                obj.lengths{t} = obj.kinematicsObj.cableLengths;
                obj.lengths_dot{t} = obj.kinematicsObj.cableLengthsDot;
            end
        end
    end
end

