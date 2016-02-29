classdef WrenchFeasible < WorkspaceConditionBase
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        desired_wrench_set
    end
    
    methods
        %% Constructor for wrench closure workspace
        function w = WrenchFeasible(method,desired_wrench_set)
            if(nargin>1)
                if(strcmp(method,'capacity_margin'))
                    w.method = WrenchFeasibleMethods.CM;
                else
                    msg = 'Incorrect wrench method set';
                    error(msg);
                end
                w.desired_wrench_set = desired_wrench_set;
            % Translate the method into an enum
            else
                w.method = WrenchFeasibleMethods.CM;
            end 
            
        end
        
        %% Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluateFunction(obj,dynamics)
           if(obj.method == WrenchFeasibleMethods.CM)
               inWorkspace = wrench_feasible_capacity_margin(obj.desired_wrench_set,dynamics);
           end
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6; l_x = size(workspace,1) - 1;
            isConnected = sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

