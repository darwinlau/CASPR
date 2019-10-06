% The simulator to run an inverse kinematics simulation
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   The inverse kinematics simulator simply compute the cable lengths for a
%   specified joint space trajectory. This is trivial for CDPRs (as a type
%   of parallel robots) and only requires the "update" function of the
%   SystemModel to be called.
classdef InverseKinematicsSimulator < MotionSimulatorBase
    methods
        % Constructors
        function ik = InverseKinematicsSimulator(model)
            ik@MotionSimulatorBase(model);
        end
        
        % Implementation of the run function.
        function run(obj, trajectory, cable_indices)
            if (nargin <= 2 || isempty(cable_indices))
                cable_indices = 1:obj.model.numCables;
            end
            
            obj.trajectory = trajectory;            
            obj.timeVector = obj.trajectory.timeVector;
            
            % Runs the simulation over the specified trajectory
            obj.cableLengths = cell(1, length(obj.trajectory.timeVector));
            obj.cableLengthsDot = cell(1, length(obj.trajectory.timeVector));
            
            CASPR_log.Info('Begin inverse kinematics simulator run...');
            for t = 1:length(obj.trajectory.timeVector)
                CASPR_log.Print(sprintf('Time : %f', obj.trajectory.timeVector(t)),CASPRLogLevel.INFO);
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_dot{t})));
                obj.cableLengths{t} = obj.model.cableLengths(cable_indices);
                obj.cableLengthsDot{t} = obj.model.cableLengthsDot(cable_indices);
            end
        end
    end
end