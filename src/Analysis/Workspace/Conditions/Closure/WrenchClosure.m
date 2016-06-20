% Class to compute whether a pose (dynamics) is within the wrench-closure
% workspace (WCW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
classdef WrenchClosure < WorkspaceConditionBase
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchClosure(method)
            w.options               =   optimset('display','off');
            if(isempty(method))
                w.method = [];
            else
                w.method = method; 
            end 
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluateFunction(obj,dynamics,workspace_point)
            if(isempty(obj.method))
                inWorkspace = obj.metrics_evaluation(dynamics,workspace_point);
            else
                switch(obj.method)
                case WrenchClosureMethods.M_QUAD_PROG
                    inWorkspace = wrench_closure_quadprog(dynamics,obj.options);
                case WrenchClosureMethods.M_TENSION_FACTOR
                    inWorkspace = wrench_closure_tension_factor(dynamics,obj.options);
                case WrenchClosureMethods.M_UNILATERAL_DEXTERITY
                    inWorkspace = wrench_closure_unilateral_dexterity(dynamics,obj.options);
                case WrenchClosureMethods.M_COMBINATORIC_NULL_SPACE
                    inWorkspace = wrench_closure_combinatoric_null_space(dynamics,obj.options);
                case WrenchClosureMethods.M_COMBINATORIC_POSITIVE_SPAN
                    inWorkspace = wrench_closure_combinatoric_positive_span(dynamics,obj.options);
                otherwise
                    error('Wrench closure method is not defined');
                end
            end
        end
    end
    
    methods (Access = private)
        % A method to evaluate the workspace condition using already known
        % metric information
        function inWorkspace = metrics_evaluation(obj,dynamics,workspace_point)
            % Check if there is a metric that can already be used.
            for i = 1:length(workspace_point.metrics)
                % Options are 1 if in workspace, 0 if not in workspace,
                % -1 if metric doesn't give workspace information.
                gm = obj.generating_metric({workspace_point.metrics{i,:}}); 
                if(gm==1)
                    inWorkspace = true;
                    return;
                elseif(gm==0)
                    inWorkspace = false;
                    return;
                end
            end
            inWorkspace = wrench_closure_quadprog(dynamics,obj.options);
        end
        
        % A use the metric for evaluation if it is there
        function val = generating_metric(~,metric)
            switch(metric{1})
                case {WorkspaceMetricType.TENSION_FACTOR,WorkspaceMetricType.TENSION_FACTOR_MODIFIED,WorkspaceMetricType.UNILATERAL_DEXTERITY}
                    if(metric{2}>0)
                        val = 1;
                    else
                        val = 0;
                    end
                otherwise
                    % The metric cannot be used for the evaluation
                    val = -1;
            end
        end        
    end
end

