classdef Metric < handle
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluate(obj, dynamics);
        % workspaceCheck - This function determines if the metric and
        % workspace are compatible such that they can be computed at once.
        f = workspaceCheck(obj,type)        
    end
end