classdef Metric < handle
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluate(obj, dynamics);
    end
end