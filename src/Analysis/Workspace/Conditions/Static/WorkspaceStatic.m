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
            if(nargin>0)
                if(strcmp(method,'quad_prog'))
                    w.method = WorkspaceStaticMethods.QP;
                elseif(strcmp(method,'capacity_margin'))
                    w.method = WorkspaceStaticMethods.CMa;
                elseif(strcmp(method,'capability_measure'))
                    w.method = WorkspaceStaticMethods.CMe;
                else
                    msg = 'Incorrect static workspace method set';
                    error(msg);
                end
            % Translate the method into an enum
            else
                w.method = WorkspaceStaticMethods.QP;
            end 
        end
        
        % The implementation of the evaluateFunction method
        function inWorkspace = evaluateFunction(obj,dynamics)
           if(obj.method == WorkspaceStaticMethods.QP)
               inWorkspace = static_quadprog(dynamics,obj.options);
           elseif(obj.method == WorkspaceStaticMethods.CMa)
               inWorkspace = static_capacity_margin(dynamics,obj.options);
           else
               inWorkspace = static_capability_measure(dynamics,obj.options);
           end
        end
        
        % The implementation of the connected method
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % This file may need a dynamics object added at a later date
            tol = 1e-6;
            isConnected = sum((abs(workspace(:,i) - workspace(:,j)) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
        
        % A function to be used to set options.
        function setOptions(obj,options)
            obj.options = options;
        end
    end
end