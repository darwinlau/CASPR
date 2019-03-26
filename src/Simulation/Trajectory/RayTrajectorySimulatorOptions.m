% The options for running a ray-based trajectory simulator
%
% Author        : Zeqing ZHANG
% Created       : 2019
% Description   :
% Trajectory simulator options.  To be modified over time

classdef RayTrajectorySimulatorOptions
    properties
        union           % Determines if the trajectory should take a union of the conditions or the intersection
        read_mode       % Determines if the simulator is in read mode
        slices          % Store the info. about which slices of which degrees would be investigated
    end
    
    methods
        function opt = RayTrajectorySimulatorOptions(union,read_mode,slices)
            opt.union       = union;
            opt.read_mode   = read_mode;
            if (nargin<3)
                opt.slices = [];
            else
                opt.slices  = slices;
            end
        end
    end
end

