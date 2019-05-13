% Class to compute whether a pose (dynamics) is within the wrench feasible
% workspace (WFW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : Class for evalaution of WFC.
classdef WrenchFeasibleCondition < WorkspaceConditionBase
    properties (Constant)
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceConditionType.WRENCH_FEASIBLE;
    end
    
    properties (SetAccess = protected, GetAccess = protected)
        desired_wrench_set
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchFeasibleCondition(method, desired_wrench_set)
            if(isempty(method))
                w.method = WrenchFeasibleMethodType.M_CAPACITY_MARGIN;
            else
                w.method = method; 
            end 
            w.desired_wrench_set = desired_wrench_set;
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluateFunction(obj, dynamics, evaluated_metrics)
            % Check if there's any metrics that are already evaluated that 
            % can help to solve for the workspace condition
            if (~isempty(evaluated_metrics))
                for i = 1:size(evaluated_metrics, 1)  
                    if (evaluated_metrics{i, 1}.type == WorkspaceMetricType.CAPACITY_MARGIN && isequal(obj.desired_wrench_set, evaluated_metrics{i, 1}.desired_wrench_set))
                        if (evaluated_metrics{i, 2} >= 0)
                            inWorkspace = true;
                            return;
                        else
                            inWorkspace = false;
                            return
                        end
                    end
                end
            end
            switch(obj.method)
                case WrenchFeasibleMethod.M_CAPACITY_MARGIN
                    inWorkspace = wrench_feasible_capacity_margin(obj.desired_wrench_set,dynamics);
                otherwise
                    CASPR_log.Print('Wrench feasible method is not defined',CASPRLogLevel.ERROR);
            end
        end
    end
end