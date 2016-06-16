% Class to compute whether a pose (dynamics) is within the wrench feasible
% workspace (WFW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : Class for evalaution of WFC.
classdef WrenchFeasible < WorkspaceCondition
    properties (SetAccess = protected, GetAccess = protected)
        desired_wrench_set
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchFeasible(method,desired_wrench_set)
            if(isempty(method))
                w.method = [];
            else
                w.method = method; 
            end 
            w.desired_wrench_set = desired_wrench_set;
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluateFunction(obj,dynamics)
            switch(method)
                case WrenchFeasibleMethods.M_CAPACITY_MARGIN
                    inWorkspace = wrench_feasible_capacity_margin(obj.desired_wrench_set,dynamics);
                otherwise
                    error('Wrench feasible method is not defined');
            end
        end
    end
end

