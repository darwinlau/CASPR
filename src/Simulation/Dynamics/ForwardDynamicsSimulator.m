% The simulator to run a forward dynamics simulation
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :
%   The forward dynamics simulator solves for the generalised coordinates
%   for a given cable-force trajectory. The FD solver that should be used
%   to resolve and integrate the EoM is specified to the simulator.
classdef ForwardDynamicsSimulator < DynamicsSimulator    
    properties        
        fdSolver
    end
    
    methods
        % Constructor for the forward dynamics
        function fd = ForwardDynamicsSimulator(model, fd_solver_type)
            fd@DynamicsSimulator(model);
            fd.fdSolver = ForwardDynamics(fd_solver_type);
        end
        
        % Implementation of the run function
        function run(obj, cable_forces_active, cable_indices_active, time_vector, q0, q0_dot)
            obj.timeVector = time_vector;
            obj.cableForces = cable_forces_active;
            
            obj.trajectory = JointTrajectory;
            obj.trajectory.timeVector = obj.timeVector;
            obj.trajectory.q = cell(1, length(obj.timeVector));
            obj.trajectory.q_dot = cell(1, length(obj.timeVector));
            obj.trajectory.q_ddot = cell(1, length(obj.timeVector));
            
            % Setup initial pose
            obj.model.update(q0, q0_dot, zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1));
            q0_ddot = obj.model.q_ddot_dynamics;
            obj.model.update(q0, q0_dot, q0_ddot, zeros(obj.model.numDofs,1));
            
            obj.trajectory.q{1} = q0;
            obj.trajectory.q_dot{1} = q0_dot;
            obj.trajectory.q_ddot{1} = q0_ddot;
            
            for t = 2:length(obj.timeVector)
                CASPR_log.Print(sprintf('Simulation time : %f', obj.timeVector(t)),CASPRLogLevel.INFO);
                [obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t}, obj.model] = obj.fdSolver.compute(obj.model.q, obj.model.q_dot, cable_forces_active{t-1}, cable_indices_active{t-1}, zeros(obj.model.numDofs,1), obj.timeVector(t)-obj.timeVector(t-1), obj.model);
                obj.interactionWrench{t} = obj.model.interactionWrench;
                obj.cableLengths{t} = obj.model.cableLengths;
                obj.cableLengthsDot{t} = obj.model.cableLengthsDot;
            end
        end        
    end
end