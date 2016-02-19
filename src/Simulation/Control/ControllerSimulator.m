classdef ControllerSimulator < DynamicsSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)

    properties (SetAccess = protected)
        compTime            % computational time for each time step
        controller
        refTrajectory
        stateError
    end

    methods
        function ctrl_sim = ControllerSimulator(model, controller)
            ctrl_sim@DynamicsSimulator(model);
            ctrl_sim.model = model;
            ctrl_sim.controller = controller;
        end

        function run(obj, ref_trajectory, q0, q0_dot, q0_ddot)
            obj.refTrajectory = ref_trajectory;
            obj.timeVector = obj.refTrajectory.timeVector;
            obj.stateError = cell(1, length(obj.timeVector));
            obj.cableForces = cell(1, length(obj.timeVector));
            
            obj.trajectory = JointTrajectory;
            obj.trajectory.timeVector = obj.timeVector;
            obj.trajectory.q = cell(1, length(obj.timeVector));
            obj.trajectory.q_dot = cell(1, length(obj.timeVector));
            obj.trajectory.q_ddot = cell(1, length(obj.timeVector));
            
            obj.trajectory.q{1} = q0;
            obj.trajectory.q_dot{1} = q0_dot;
            obj.trajectory.q_ddot{1} = q0_ddot;
            
            for t = 1:length(obj.timeVector)
                fprintf('Time : %f\n', obj.timeVector(t));
                obj.cableForces{t} = obj.controller.executeFunction(obj.trajectory.q{t},  obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t}, ref_trajectory.q{t}, ref_trajectory.q_dot{t}, ref_trajectory.q_ddot{t});
                obj.stateError{t} = ref_trajectory.q{t} - obj.trajectory.q{t};
                if t < length(obj.timeVector)
                    [obj.trajectory.q{t+1}, obj.trajectory.q_dot{t+1}, obj.trajectory.q_ddot{t+1}, obj.model] = ForwardDynamics.Compute(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.cableForces{t}, obj.timeVector(t+1)-obj.timeVector(t), obj.model);
                end
            end
        end
        
        function plotTrackingError(obj)
            trackingError_array = cell2mat(obj.stateError);
            plot(obj.timeVector, trackingError_array, 'Color', 'k', 'LineWidth', 1.5);
        end
    end
end