% The simulator to run a workspace simulation
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
%   Workspace simulator generates the workspace over a defined set of space
%   (currently only a grid of states is accepted). The workspace simulation
%   performs essentially a numerical brute force approach over the set of
%   poses for a given workspace condition and/or metric.
classdef PointWorkspaceSimulator < SimulatorBase
    
    properties
        grid                        % Grid object for brute force workspace (input)
        workspace                   % Final Workspace (output)
    end
    
    properties (SetAccess = private)
        compTime                    % Total time to compute workspace
        conditions = []             % An array of conditions to be evaluated for
        metrics = []                % An array of metrics to be evaluated for
        connectivity                % Connectivity condition (children of WorkspaceConnectivityBase object)
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = PointWorkspaceSimulator(model, grid, conditions, metrics, connectivity)
            w@SimulatorBase(model);
            w.grid          = grid;
            w.conditions    = conditions;
            w.metrics       = metrics;
            w.connectivity  = connectivity;
        end
        
        % Implementation of the run function
        function run(obj)
            obj.compTime = 0;
            for i = 1:size(obj.metrics,2)
                if((~isempty(obj.metrics{i}.metricMax))&&((abs(obj.metrics{i}.metricMax)==Inf)||(abs(obj.metrics{i}.metricMin)==Inf)))
                    CASPR_log.Print('A metric with infinite limit values cannot be plotted.  To plot please set the metric limit to be finite or filter the workspace after plotting',CASPRLogLevel.WARNING);
                end
            end
            
            n_grid_points = obj.grid.n_points;
            
            % Instantiate workspace object
            obj.workspace = PointWorkspace(obj.model, obj.conditions, obj.metrics, obj.grid);

            workspace_count = 0;
            
            % Runs over the grid and evaluates the workspace condition at
            % each point
            log_level = CASPRLogLevel.DEBUG;
            is_log = (log_level >= CASPR_log.GetLogLevel());
            
            for i = 1:n_grid_points
                if (is_log)
                    CASPR_log.Print(sprintf('Workspace point %d. Completion Percentage: %3.2f', i,100*i/n_grid_points), log_level);
                end
                % Get the grid point
                q = obj.grid.getGridPoint(i);
                % Construct and evaluate the workspace point
                wp = PointWorkspaceElement(q, obj.model, obj.conditions, obj.metrics);
                               
                if (~isempty(wp.conditions) || ~isempty(wp.metrics))
                    workspace_count = workspace_count + 1;
                    obj.workspace.poses{workspace_count} = wp;
                end
                obj.compTime = obj.compTime + wp.compTime;
            end
            obj.workspace.poses = obj.workspace.poses(1:workspace_count, 1);
        end  
    end
end
