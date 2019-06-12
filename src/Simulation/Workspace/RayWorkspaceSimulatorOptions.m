% The options for running a workspace simulator
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
% Workspace simulator options.  To be modified over time

classdef RayWorkspaceSimulatorOptions
    properties
        union           % Determines if the workspace should take a union of the conditions or the intersection
%         read_mode       % Determines if the simulator is in read mode
%         slices          % Store the info. about which slices of which degrees would be investigated
    end
    
    methods
        function opt = RayWorkspaceSimulatorOptions(union)
            opt.union       = union;
%             opt.read_mode   = read_mode;
%             if (nargin<3)
%                 opt.slices = [];
%             else
%                 opt.slices  = slices;
%             end
        end
    end
end

