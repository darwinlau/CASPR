% Class to compute whether a pose (dynamics) is within the interference free workspace
% (IFW)
%
% Author        : Zeqing ZHANG
% Created       : 2017
% Description   : The class for evaluation of IFW  
classdef InterferenceFreeCondition < WorkspaceConditionBase    
    properties (Constant)
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceConditionType.INTERFERENCE;
    end
    
    properties
        min_cable_dist              % minimum distance for cable-cable interference free
    end
    
    methods
        % Constructor for Interference Free WS
        function w = InterferenceFreeCondition(method, cable_d)
            if (isempty(method))
                w.method = InterferenceFreeMethodType.M_MINDISTANCE_CABLE_CABLE;
            else
                w.method = method;
            end
            w.min_cable_dist = cable_d;
        end
        
        % Evaluate the Interference Free condition return true if
        % satisified
        function inWorkspace = evaluateFunction(obj, dynamics, evaluated_metrics)
            % Check if there's any metrics that are already evaluated that 
            % can help to solve for the workspace condition
            if (~isempty(evaluated_metrics))
                for i = 1:size(evaluated_metrics, 1)  
                    if (evaluated_metrics{i, 1}.type == WorkspaceMetricType.MIN_CABLE_CABLE_DISTANCE)
                        inWorkspace = (evaluated_metrics{i, 2} > obj.min_cable_dist);
                        return;
                    end
                end
            end
            switch(obj.method)
                case InterferenceFreeMethodType.M_MINDISTANCE_CABLE_CABLE
                    inWorkspace = interference_free_mindistance_cable_cable(dynamics, obj.min_cable_dist);
                otherwise
                    CASPR_log.Print('Interference free method is not defined',CASPRLogLevel.ERROR);
            end
        end
    end 
end

