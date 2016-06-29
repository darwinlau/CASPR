% Class to compute whether a pose (dynamics) is within the static workspace
% (SW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : This class is the class for static workspace evaluation.
% Different implementations are treated as individual function calls for the
% evaluate function.
classdef WorkspaceStatic < WorkspaceConditionBase
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        % The constructor for this class.
        function w = WorkspaceStatic(method)
            w.options               =   optimset('display','off','Algorithm','interior-point-convex');
            if(isempty(method))
                w.method = [];
            else
                w.method = method; 
            end 
        end
        
        % The implementation of the evaluateFunction method
        function inWorkspace = evaluateFunction(obj,dynamics,workspace_point)
            if(isempty(obj.method))
                inWorkspace = obj.metrics_evaluation(dynamics,workspace_point);
            else
                switch(method)
                    case WorkspaceStaticMethods.M_QUAD_PROG
                        inWorkspace = static_quadprog(dynamics,obj.options);
                    case WorkspaceStaticMethods.M_CAPACITY_MARGIN
                        inWorkspace = static_capacity_margin(dynamics,obj.options);
                    case WorkspaceStaticMethods.M_SEACM
                        inWorkspace = static_capability_measure(dynamics,obj.options);
                    otherwise
                        CASPR_log.Print('static workspace method is not defined',CASPRLogLevel.ERROR);
                end
            end
        end
        
        % A function to be used to set options.
        function setOptions(obj,options)
            obj.options = options;
        end
    end
    
    methods (Access = private)
        % A method to evaluate the workspace condition using already known
        % metric information
        function inWorkspace = metrics_evaluation(obj,dynamics,workspace_point)
            % Check if there is a metric that can already be used.
            for i = 1:size(workspace_point.metrics,1)
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
            inWorkspace = static_quadprog(dynamics,obj.options);
        end
        
        % A use the metric for evaluation if it is there
        function val = generating_metric(~,metric)
            if(~isempty(metric{1}))
                switch(metric{1})
                    case WorkspaceMetricType.SEACM
                        if(metric{2}>=0)
                            val = 1;
                        else
                            val = 0;
                        end
                    otherwise
                        % The metric cannot be used for the evaluation
                        val = -1;
                end
            else
                val = -1;
            end
        end        
    end
end