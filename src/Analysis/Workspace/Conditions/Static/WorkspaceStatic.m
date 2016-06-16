% Class to compute whether a pose (dynamics) is within the static workspace
% (SW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : This class is the class for static workspace evaluation.
% Different implementations are treated as individual function calls for the
% evaluate function.
classdef WorkspaceStatic < WorkspaceCondition
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
        function inWorkspace = evaluateFunction(obj,dynamics)
            switch(method)
                case WorkspaceStaticMethods.M_QUAD_PROG
                    inWorkspace = static_quadprog(dynamics,obj.options);
                case WorkspaceStaticMethods.M_CAPACITY_MARGIN
                    inWorkspace = static_capacity_margin(dynamics,obj.options);
                case WorkspaceStaticMethods.M_SEACM
                    inWorkspace = static_capability_measure(dynamics,obj.options);
                otherwise
                    error('static workspace method is not defined');
            end
        end
        
        % A function to be used to set options.
        function setOptions(obj,options)
            obj.options = options;
        end
    end
end