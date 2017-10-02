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
classdef ForwardKinematicsSimulatorSet < handle
    
    properties (SetAccess = private)
        simulators 
        numSims
    end
    
    methods
        % Constructor for the forward kinematics class
        function fk = ForwardKinematicsSimulatorSet(model, fk_solvers)
            fk.numSims = length(fk_solvers);
            for i = 1:fk.numSims
                fk.simulators{i} = ForwardKinematicsSimulator(model, fk_solvers{i});
            end
        end
        
        % The run function performs the FK at each point in time using the
        % trajectory of cable lengths
        function run(obj, lengths, lengths_dot, time_vector, q0_approx, q0_prev_approx)
            for i = 1:obj.numSims
                obj.simulators{i}.run(lengths, lengths_dot, time_vector, q0_approx, q0_prev_approx);
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

