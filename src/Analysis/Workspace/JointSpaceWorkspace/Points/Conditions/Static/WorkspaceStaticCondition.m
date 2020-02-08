% Class to compute whether a pose (dynamics) is within the static workspace
% (SW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : This class is the class for static workspace evaluation.
% Different implementations are treated as individual function calls for the
% evaluate function.
classdef WorkspaceStaticCondition < WorkspaceConditionBase
    properties (Constant)
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceConditionType.STATIC;
    end
    
    properties
        options                         % The options for the wrench closure
    end
    
    methods
        % The constructor for this class.
        function w = WorkspaceStaticCondition(method)
            w.options               =   optimset('display','off','Algorithm','interior-point-convex');
            if(nargin < 1 || isempty(method))
                w.method = WorkspaceStaticMethodType.M_QUAD_PROG;
            else
                w.method = method; 
            end 
        end
        
        % The implementation of the evaluateFunction method
        function inWorkspace = evaluateFunction(obj, dynamics, evaluated_metrics)                    
            % Check if there's any metrics that are already evaluated that 
            % can help to solve for the workspace condition
            if (~isempty(evaluated_metrics))
                for i = 1:size(evaluated_metrics, 1)  
                    if (evaluated_metrics{i, 1}.type == WorkspaceMetricType.SEACM)
                        inWorkspace = (evaluated_metrics{i, 2} >= 0);
                        return;
                    end
                end
            end
            switch(obj.method)
                case WorkspaceStaticMethodType.M_QUAD_PROG
                    inWorkspace = static_quadprog(dynamics);
                case WorkspaceStaticMethodType.M_CAPACITY_MARGIN
                    inWorkspace = static_capacity_margin(dynamics);
                case WorkspaceStaticMethodType.M_SEACM
                    inWorkspace = static_capability_measure(dynamics);
                otherwise
                    CASPR_log.Error('Static workspace method is not defined');
            end
        end
    end
end