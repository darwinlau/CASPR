% The options for running a workspace simulator
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
% Workspace simulator options.  To be modified over time

classdef WorkspaceSimulatorOptions
    properties
        full_storage        % An option to store all grid points.
    end
    
    methods
        function opt = WorkspaceSimulatorOptions(full_storage)
            opt.full_storage = full_storage;
        end
    end
end

