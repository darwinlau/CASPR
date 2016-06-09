% The simulator to run an inverse kinematics simulation
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   The inverse kinematics simulator simply compute the cable lengths for a
%   specified joint space trajectory. This is trivial for CDPRs (as a type
%   of parallel robots) and only requires the "update" function of the
%   SystemModel to be called.
classdef InverseKinematicsSimulator < MotionSimulator
    methods
        % Constructors
        function ik = InverseKinematicsSimulator(model)
            ik@MotionSimulator(model);
        end
        
        % Implementation of the run function.
        function run(obj, trajectory)
            obj.trajectory = trajectory;
            
            obj.timeVector = obj.trajectory.timeVector;
            
            % Runs the simulation over the specified trajectory
            obj.cableLengths = cell(1, length(obj.trajectory.timeVector));
            obj.cableLengthsDot = cell(1, length(obj.trajectory.timeVector));
            
            for t = 1:length(obj.trajectory.timeVector)
                fprintf('Time : %f\n', obj.trajectory.timeVector(t));
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_dot{t})));
                obj.cableLengths{t} = obj.model.cableLengths;
                obj.cableLengthsDot{t} = obj.model.cableLengthsDot;
            end
        end
    end
end

