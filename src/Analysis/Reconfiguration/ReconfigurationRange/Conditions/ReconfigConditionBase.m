% Base class for different ray reconfig conditions
%
% Author        : Paul Cheng
% Created       : 2020
% Description:

classdef ReconfigConditionBase < handle
    properties (Constant, Abstract)
        type            % Type of workspace condition (WorkspaceRayConditionType enum)
    end
    
    properties (SetAccess = protected)
        method              % Method of implementation (an enum)
    end
    
    methods
        function w = ReconfigConditionBase()
            
        end
        
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        % Resulting condition_intervals is of the type
        function [condition_intervals, condition_type, comp_time] = evaluate(obj, reconfig_element)
            start_tic       = tic;
            condition_intervals = obj.evaluateFunction(reconfig_element);
            condition_type  = obj.type;
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, model, reconfig_element);
    end
    
    methods (Static)
        % Creates a new condition (for the moment methods and wrench sets
        % are not considered)
        function wc = CreateReconfigCondition(conditionType,model,obstacles)
            switch conditionType
                case ReconfigConditionType.WRENCH_CLOSURE
                    wc = WrenchClosureReconfigCondition(model);
                case ReconfigConditionType.INTERFERENCE_CABLE_OBSTACLE
                    wc = InterferenceFreeReconfigCondition(model,obstacles);
                otherwise
                    CASPR_log.Error('Workspace condition type is not defined');
            end
        end
    end
end