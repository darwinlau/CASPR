% The simulator to run a control simulation
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%   The control problem aims to solve for the cable forces in order to
%   track a reference trajectory in joint space. The controller algorithm
%   (ControllerBase object) is specified for the simulator. 
classdef ControllerSimulator < DynamicsSimulator
    properties (SetAccess = protected)
        compTime            % computational time for each time step
        fdSolver            % The forward dynamics solver
        controller          % The controller for the system
        refTrajectory       % The reference trajectory
        stateError          % The state space error
    end

    methods
        % The control simulator constructor
        function ctrl_sim = ControllerSimulator(model, controller, fd_solver)
            ctrl_sim@DynamicsSimulator(model);
            ctrl_sim.model = model;
            ctrl_sim.controller = controller;
            ctrl_sim.fdSolver = fd_solver;
        end

        % Implementation of the run function. Converts the dynamics
        % information into a controller
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
                CASPR_log.Print(sprintf('Time : %f\n', obj.timeVector(t)),CASPRLogLevel.INFO);
                [obj.cableForcesActive{t}, obj.cableIndicesActive{t}, obj.cableForces{t}] = obj.controller.execute(obj.trajectory.q{t},  obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t}, ref_trajectory.q{t}, ref_trajectory.q_dot{t}, ref_trajectory.q_ddot{t}, obj.timeVector(t));
                obj.stateError{t} = ref_trajectory.q{t} - obj.trajectory.q{t};
                if t < length(obj.timeVector)
                    [obj.trajectory.q{t+1}, obj.trajectory.q_dot{t+1}, obj.trajectory.q_ddot{t+1}, obj.model] = obj.fdSolver.compute(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.cableForcesActive{t}, obj.cableIndicesActive{t}, zeros(obj.model.numDofs,1), obj.timeVector(t+1)-obj.timeVector(t), obj.model);
                end
            end
        end
        
        % Plots the tracking error in generalised coordinates
        function plotTrackingError(obj, plot_axis)
            trackingError_array = cell2mat(obj.stateError);
            if(nargin == 1 || isempty(plot_axis)) 
                figure;
                plot(obj.timeVector, trackingError_array, 'Color', 'k', 'LineWidth', 1.5);
            else
                plot(plot_axis, obj.timeVector, trackingError_array, 'Color', 'k', 'LineWidth', 1.5);
            end
        end
        
        % Plots both the reference and computed trajectory.
        function plotJointSpaceTracking(obj, plot_axis, states_to_plot)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');

            n_dof = obj.model.numDofs;

            if nargin <= 2 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end

            q_array = cell2mat(obj.trajectory.q);
            q_dot_array = cell2mat(obj.trajectory.q_dot);
            q_ref_array = cell2mat(obj.refTrajectory.q);
            q_ref_dot_array = cell2mat(obj.refTrajectory.q_dot);

            if nargin <= 1 || isempty(plot_axis)
                % Plots joint space variables q(t)
                figure;
                hold on;
                plot(obj.timeVector, q_ref_array(states_to_plot, :), 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
                plot(obj.timeVector, q_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                hold off;
                title('Joint space variables');

                % Plots derivative joint space variables q_dot(t)
                figure;
                hold on;
                plot(obj.timeVector, q_ref_dot_array(states_to_plot, :), 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
                plot(obj.timeVector, q_dot_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                hold off;
                title('Joint space derivatives');
            else
                plot(plot_axis(1),obj.timeVector, q_ref_array(states_to_plot, :), 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'r'); 
                hold on;
                plot(plot_axis(1),obj.timeVector, q_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); 
                hold off;
                plot(plot_axis(2),obj.timeVector, q_ref_dot_array(states_to_plot, :), 'LineWidth', 'LineStyle', '--', 1.5, 'Color', 'r'); 
                hold on;
                plot(plot_axis(2),obj.timeVector, q_dot_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); 
                hold off;                
            end
        end
    end
end