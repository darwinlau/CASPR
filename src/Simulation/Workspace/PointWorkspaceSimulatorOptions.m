% The options for running a workspace simulator
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
% Workspace simulator options.  To be modified over time

classdef PointWorkspaceSimulatorOptions
    properties
        union           % Determines if the workspace should take a union of the conditions or the intersection
        solver_options  % An options object for different solvers
    end
    
    methods
        function opt = PointWorkspaceSimulatorOptions(union,solver_options)
            opt.union = union;
            opt.solver_options = solver_options;
        end
    end
end

