classdef WorkspaceMetricBase < handle
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods 
        function [f, comp_time] = evaluate(obj,dynamics,method,inWorkspace)
            start_tic = tic;
            f = obj.evaluateFunction(dynamics,method,inWorkspace);
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, dynamics,method,inWorkspace);        
    end
end