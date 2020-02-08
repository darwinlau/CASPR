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
    
    properties (SetAccess = protected)
        desiredWrenchSet
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchFeasibleCondition(desiredWrenchSet, method)
            if(nargin < 2 || isempty(method))
                w.method = WrenchFeasibleMethodType.M_CAPACITY_MARGIN;
            else
                w.method = method; 
            end 
            w.desiredWrenchSet = desiredWrenchSet;
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluateFunction(obj, dynamics, evaluated_metrics)
            % Check if there's any metrics that are already evaluated that 
            % can help to solve for the workspace condition
            if (~isempty(evaluated_metrics))
                for i = 1:size(evaluated_metrics, 1)  
                    if (evaluated_metrics{i, 1}.type == WorkspaceMetricType.CAPACITY_MARGIN && isequal(obj.desiredWrenchSet, evaluated_metrics{i, 1}.desiredWrenchSet))
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
                case WrenchFeasibleMethodType.M_CAPACITY_MARGIN
                    inWorkspace = wrench_feasible_capacity_margin(obj.desiredWrenchSet,dynamics);
                case WrenchFeasibleMethodType.M_LINEAR_PROGRAMMING_MATLAB
                    inWorkspace = wrench_feasible_linear_programming_MATLAB(obj.desiredWrenchSet,dynamics);
                case WrenchFeasibleMethodType.M_LINEAR_PROGRAMMING_CPLEX
                    inWorkspace = wrench_feasible_linear_programming_CPLEX(obj.desiredWrenchSet,dynamics);
                otherwise
                    CASPR_log.Print('Wrench feasible method is not defined', CASPRLogLevel.ERROR);
            end
        end
    end
end