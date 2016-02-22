classdef WorkspaceConditionBase < handle
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        method                    % Method of implementation
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluate(obj, dynamics);
        % connected - This function takes points in a workspace and returns
        % true if the point is connected (using discrete connectivity) to
        % another point in the workspace.
        f = connected(obj, wsim);
    end
end

