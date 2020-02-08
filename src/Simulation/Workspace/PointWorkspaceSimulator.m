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
        comp_time_total             % Total time to compute workspace
        comp_time_evaluation        % Time to just evaluate the workspace points
        comp_time_graph             % Time to construct the graph structure
        
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
            for i = 1:size(obj.metrics,2)
                if((~isempty(obj.metrics{i}.metricMax))&&((abs(obj.metrics{i}.metricMax)==Inf)||(abs(obj.metrics{i}.metricMin)==Inf)))
                    CASPR_log.Print('A metric with infinite limit values cannot be plotted.  To plot please set the metric limit to be finite or filter the workspace after plotting',CASPRLogLevel.WARNING);
                end
            end
            
            n_grid_points = obj.grid.n_points;
            
            % Instantiate workspace object
            obj.workspace = PointWorkspace(obj.model, obj.conditions, obj.grid);

            workspace_count = 0;
            
            % Timing variables
            point_t_in = tic;
            total_t_in = tic;
            empty_ws_num = [];
            % Runs over the grid and evaluates the workspace condition at
            % each point
            for i = 1:n_grid_points
                CASPR_log.Print([sprintf('Workspace point %d. ',i),sprintf('Completion Percentage: %3.2f',100*i/n_grid_points)],CASPRLogLevel.INFO);
                % Get the grid point
                q = obj.grid.getGridPoint(i);
                % Construct and evaluate the workspace point
                wp = PointWorkspaceElement(q, obj.model, obj.conditions, obj.metrics);
                               
                if (~isempty(wp.conditions) || ~isempty(wp.metrics))
                    obj.workspace.poses{i} = wp;
                    workspace_count = workspace_count + 1;
                else
                    empty_ws_num = [empty_ws_num,i];
                end
            end
            obj.workspace.poses(empty_ws_num,:) = [];
            obj.comp_time_evaluation = toc(point_t_in);
            graph_t_in = tic;
                
            obj.comp_time_graph = toc(graph_t_in);
            obj.comp_time_total = toc(total_t_in);
        end  
    end
end
