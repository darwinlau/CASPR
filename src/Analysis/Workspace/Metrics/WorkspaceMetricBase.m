% Base class for different workspace metrics
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
%   Different workspace metrics must simply implement the evaluateFunction
classdef WorkspaceMetricBase < handle
    
    properties
    end
    
    methods 
        function [f, comp_time] = evaluate(obj,dynamics,options,method,inWorkspace)
            start_tic = tic;
            f = obj.evaluateFunction(dynamics,options,method,inWorkspace);
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns the metric value
        f = evaluateFunction(obj, dynamics,options,method,inWorkspace);        
    end
end