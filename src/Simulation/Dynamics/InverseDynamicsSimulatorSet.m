% The simulator to run an inverse dynamics simulation
%
% Author        : Darwin LAU
% Created       : 2017
% Description    :
%   The inverse dynamics simulator solves for the cable forces required to
%   perform a prescribed joint space trajectory. The IDSolver is provided
%   as an input to the simulator to specify the ID algorithm to be used.
classdef InverseDynamicsSimulatorSet < handle

    properties (SetAccess = protected)
        simulators 
        numSims
    end

    methods
        % Constructor for the inverse dynamics simulator
        function id = InverseDynamicsSimulatorSet(model, id_solvers)
            id.numSims = length(id_solvers);
            for i = 1:id.numSims
                id.simulators{i} = InverseDynamicsSimulator(model, id_solvers{i});
            end
        end

        % Implementation of the run function
        function run(obj, trajectory)
            for i = 1:obj.numSims
                obj.simulators{i}.run(trajectory);
            end
        end

        % Plots the computational cost for the solver
        function plotCompCost(obj,plot_axis)
            if(nargin == 1 || isempty(plot_axis))
                figure; hold on;
                for i = 1:obj.numSims
                    plot(obj.simulators{i}.timeVector, obj.simulators{i}.compTime);
                end
                title('ID computational time');
            else
                for i = 1:obj.numSims
                    plot(plot_axis,obj.timeVector, obj.simulators{i}.compTime);
                end
            end
        end
    end
end