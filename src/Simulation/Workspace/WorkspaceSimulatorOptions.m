% The options for running a workspace simulator
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
% Workspace simulator options.  To be modified over time

classdef WorkspaceSimulatorOptions
    properties
        union       % Determines if the workspace should take a union of the conditions or the intersection
    end
    
    methods
        function opt = WorkspaceSimulatorOptions(union)
            opt.union = union;
        end
    end
end

