% The simulator to run an forward kinematics simulation
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   The forward kinematics simulator computes the generalised coordinates
%   trajectory given the cable length trajectory using the specified FK 
%   solver. The simulator also computes and stores the resulting length
%   error by comparing the input length with the length resulting from the
%   solution generalised coordinates. This can be used as a measure of the
%   accuracy of the FK approach.
classdef ForwardKinematicsSimulator < MotionSimulatorBase
    
    properties (SetAccess = protected) 
        compTime            % computational time for each time step
        lengthError         % Cell array of the length error vector
        lengthErrorNorm     % Array of the error norm for the trajectory
        FKSolver            % The FK solver object (inherits from FKAnalysisBase)
    end
    
    properties (Dependent)
        compTimeTotal       % total computational time
        lengthErrorTotal    % total length error of FK solver
    end
    
    methods
        % Constructor for the forward kinematics class
        function fk = ForwardKinematicsSimulator(model, fk_solver)
            fk@MotionSimulatorBase(model);
            fk.FKSolver = fk_solver;
        end
        
        % The run function performs the FK at each point in time using the
        % trajectory of cable lengths
        function run(obj, lengths, lengths_dot, time_vector, q0_approx, q0_dot_approx, cable_indices)
            if (nargin <= 6 || isempty(cable_indices))
                cable_indices = 1:obj.model.numCables;
            end
                
            
            obj.timeVector = time_vector;
            obj.cableLengths = lengths;
            obj.cableLengthsDot = lengths_dot;
            obj.compTime = zeros(length(obj.timeVector), 1);
            
            % Setting up
            obj.trajectory = JointTrajectory;
            obj.trajectory.timeVector = obj.timeVector;
            obj.trajectory.q = cell(1, length(obj.timeVector));
            obj.trajectory.q_dot = cell(1, length(obj.timeVector));
            obj.lengthError = cell(1, length(obj.timeVector));
            obj.lengthError(:) = {zeros(size(lengths{1}))};
            obj.lengthErrorNorm = zeros(length(obj.timeVector), 1);
            % Does not compute q_ddot (set it to be empty)
            obj.trajectory.q_ddot = cell(1, length(obj.timeVector));
            obj.trajectory.q_ddot(:) = {zeros(size(q0_approx))};
            
            % Runs the simulation over the specified trajectory
            q_prev = q0_approx;
            q_d_prev = q0_dot_approx;
            lengths_prev = lengths{1};
            
            time_prev = 0;
            
            CASPR_log.Info('Begin forward kinematics simulator run...');
            for t = 1:length(obj.trajectory.timeVector)
                CASPR_log.Info(sprintf('Time : %f', obj.trajectory.timeVector(t)));
                [q, q_dot, obj.compTime(t)] = obj.FKSolver.compute(lengths{t}, lengths_prev, q_prev, q_d_prev, obj.trajectory.timeVector(t) - time_prev, cable_indices);
                obj.trajectory.q{t} = q;
                obj.trajectory.q_dot{t} = q_dot;                
                q_prev = q;
                q_d_prev = q_dot;
                time_prev =  obj.trajectory.timeVector(t);
                lengths_prev = lengths{t};
                obj.lengthError{t} = obj.FKSolver.ComputeLengthErrorVector(q, lengths{t}, obj.model, cable_indices);
                obj.lengthErrorNorm(t) = norm(obj.lengthError{t});
            end
        end
        
        % Plots the error for each cable length by comparing the reference
        % length with the length as a result of the solution generalised
        % coordinates.
        function plotCableLengthError(obj, plot_axis)
            lengthError_array = cell2mat(obj.lengthError);
            if (nargin <= 1 || isempty(plot_axis))
                figure;
                plot(obj.timeVector, lengthError_array, 'Color', 'k', 'LineWidth', 1.5);
                title('Cable Length Error');
            else
                plot(plot_axis, obj.timeVector, lengthError_array, 'Color', 'k', 'LineWidth', 1.5);
            end
            xlabel('Time (seconds)')
            ylabel('Error (m)');
        end
        
        % Plots the error for each cable length by comparing the reference
        % length with the length as a result of the solution generalised
        % coordinates.
        function plotCableLengthErrorNorm(obj, plot_axis)
            if (nargin <= 1 || isempty(plot_axis))
                figure;
                plot(obj.timeVector, obj.lengthErrorNorm, 'Color', 'k', 'LineWidth', 1.5);
                title('Cable Length Error');
            else
                plot(plot_axis, obj.timeVector, obj.lengthErrorNorm, 'Color', 'k', 'LineWidth', 1.5);
            end
            xlabel('Time (seconds)')
            ylabel('Error norm (m)');
        end
        
        % Dependent variables
        function value = get.compTimeTotal(obj)
            value = sum(obj.compTime);
        end
        function value = get.lengthErrorTotal(obj)
            value = sum(obj.lengthErrorNorm);
        end
                
        function guiPlotCableLengthError(obj, plot_axis)
            obj.plotCableLengthError(plot_axis);
        end
    end
end

