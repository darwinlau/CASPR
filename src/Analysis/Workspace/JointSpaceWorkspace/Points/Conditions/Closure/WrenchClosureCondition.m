% Class to compute whether a pose (dynamics) is within the wrench-closure
% workspace (WCW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
classdef WrenchClosureCondition < WorkspaceConditionBase
    properties (Constant)
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceConditionType.WRENCH_CLOSURE;
    end
    
    properties
        options                         % The options for the wrench closure
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchClosureCondition(method)
            w.options               =   optimset('display','off');
            if(nargin < 1 || isempty(method))
                % default method
                w.method = WrenchClosureMethodType.M_QUAD_PROG;
            else
                w.method = method; 
            end 
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluateFunction(obj, dynamics, evaluated_metrics)
            % Check if there's any metrics that are already evaluated that 
            % can help to solve for the workspace condition
            if (~isempty(evaluated_metrics))
                for i = 1:size(evaluated_metrics, 1)  
                    if (evaluated_metrics{i, 1}.type == WorkspaceMetricType.TENSION_FACTOR || ...
                            evaluated_metrics{i, 1}.type == WorkspaceMetricType.TENSION_FACTOR_MODIFIED || ...
                            evaluated_metrics{i, 1}.type == WorkspaceMetricType.UNILATERAL_DEXTERITY)
                        inWorkspace = (evaluated_metrics{i, 2} > 0);
                        return;
                    end
                end
            end
            switch(obj.method)
                case WrenchClosureMethodType.M_QUAD_PROG
                    inWorkspace = wrench_closure_quadprog(dynamics, obj.options);
                case WrenchClosureMethodType.M_TENSION_FACTOR
                    inWorkspace = wrench_closure_tension_factor(dynamics);
                case WrenchClosureMethodType.M_UNILATERAL_DEXTERITY
                    inWorkspace = wrench_closure_unilateral_dexterity(dynamics);
                case WrenchClosureMethodType.M_COMBINATORIC_NULL_SPACE
                    inWorkspace = wrench_closure_combinatoric_null_space(dynamics);
                case WrenchClosureMethodType.M_COMBINATORIC_POSITIVE_SPAN
                    inWorkspace = wrench_closure_combinatoric_positive_span(dynamics);
                otherwise
                    CASPR_log.Error('Wrench closure method is not defined');
            end
        end
    end
end

