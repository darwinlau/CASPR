% Base class for different workspace connectivity conditions
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description:
%   All user-defined connectivity conditions should implement this base 
%   class and define he method to determine if neighbouring points are
%   connected.
%   Any new types of connectivity conditions need to be added to the 
%   WorkspaceConnectivityType enum and also added to the 
%   CreateWorkspaceConnectivityCondition method.
classdef WorkspaceConnectivityBase < handle
    properties 
        type            % Type of workspace connectivity
    end
    
    methods
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        function [connection_type, condition_value, comp_time] = evaluate(obj,dynamics,workspace_point_1,workspace_point_2)
            start_tic       = tic;
            condition_value = obj.evaluateFunction(dynamics, workspace_point_1,workspace_point_2);
            connection_type  = obj.type;
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, dynamics, workspace_point_1,workspace_point_2);
    end
    
    methods (Static)
        % Creates a new condition
        function wcc = CreateWorkspaceConnectivityCondition(connection_type,grid)
            switch connection_type
                case WorkspaceConnectivityType.GRID
                    wcc = UniformGridNeighbour(grid);
                otherwise
                    CASPR_log.Print('Workspace connectivity type is not defined',CASPRLogLevel.ERROR);
            end
            wcc.type = connection_type;
        end
    end
end