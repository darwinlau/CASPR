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
classdef ForwardKinematicsSimulator < MotionSimulator
    
    properties (SetAccess = protected) 
        lengthError         % Cell array of the length error vector
        lengthErrorNorm     % Array of the error norm for the trajectory
        FKSolver            % The FK solver object (inherits from FKAnalysisBase)
    end
    
    methods
        function fk = ForwardKinematicsSimulator(model, fk_solver)
            fk@MotionSimulator(model);
            fk.FKSolver = fk_solver;
        end
        
        % The run function performs the FK at each point in time using the
        % trajectory of cable lengths
        function run(obj, lengths, lengths_dot, time_vector, q0_approx, q0_prev_approx)
            obj.timeVector = time_vector;
            obj.cableLengths = lengths;
            obj.cableLengthsDot = lengths_dot;
            
            % Setting up
            obj.trajectory = JointTrajectory;
            obj.trajectory.timeVector = obj.timeVector;
            obj.trajectory.q = cell(1, length(obj.timeVector));
            obj.trajectory.q_dot = cell(1, length(obj.timeVector));
            obj.lengthError = cell(1, length(obj.timeVector));
            obj.lengthError(:) = {zeros(size(q0_approx))};
            obj.lengthErrorNorm = zeros(length(obj.timeVector), 1);
            % Does not compute q_ddot (set it to be empty)
            obj.trajectory.q_ddot = cell(1, length(obj.timeVector));
            obj.trajectory.q_ddot(:) = {zeros(size(q0_approx))};
            
            % Runs the simulation over the specified trajectory
            q_prev = q0_approx;
            q_d_prev = q0_prev_approx;
            lengths_prev = lengths{1};
            lengths_prev_2 = lengths{1};
            
            time_prev = 0;
            
            for t = 1:length(obj.trajectory.timeVector)
                fprintf('Time : %f\n', obj.trajectory.timeVector(t));
                [q, q_dot] = obj.FKSolver.compute(lengths{t}, lengths_prev_2, q_prev, q_d_prev, obj.trajectory.timeVector(t) - time_prev);
                obj.trajectory.q{t} = q;
                obj.trajectory.q_dot{t} = q_dot;
                
                q_prev = q;
                q_d_prev = q_dot;
                time_prev =  obj.trajectory.timeVector(t);
                lengths_prev_2 = lengths_prev;
                lengths_prev = lengths{t};
                obj.lengthError{t} = obj.FKSolver.ComputeLengthErrorVector(q, lengths{t}, obj.model);
                obj.lengthErrorNorm(t) = norm(obj.lengthError{t});
            end
        end
        
        % Plots the error for each cable length by comparing the reference
        % length with the length as a result of the solution generalised
        % coordinates.
        function plotCableLengthError(obj,~,plot_axis)
            lengthError_array = cell2mat(obj.lengthError);
            if(~isempty(plot_axis))
                plot(plot_axis,obj.timeVector, lengthError_array, 'Color', 'k', 'LineWidth', 1.5);
            else
                figure;
                plot(obj.timeVector, lengthError_array, 'Color', 'k', 'LineWidth', 1.5);
                title('Cable Length Error');
            end
        end
    end
end

