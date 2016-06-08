% Base class for different workspace conditions
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
%   Different workspace conditions must simply implement the
%   evaluateFunction and connected methods
classdef WorkspaceConditionBase < handle
    
    properties 
        method                    % Method of implementation (an enum)
    end
    
    methods
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        function [f, comp_time] = evaluate(obj,dynamics)
            start_tic = tic;
            f = obj.evaluateFunction(dynamics);
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, dynamics);
        % connected - This function takes points in a workspace and returns
        % true if the point is connected (using discrete connectivity) to
        % another point in the workspace.
        f = connected(obj, wsim);
    end
end

