% The simulator to run a workspace simulation for rays
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%   Workspace simulator generates the workspace over a defined set of space
%   (currently only a grid of states is accepted). The workspace is created
%   using the ray method
classdef RayWorkspaceSimulator < SimulatorBase
    
    properties
        grid            % Grid object for brute force workspace (input)
        workspace       % Final Workspace (output)
        conditions = [] % A list of conditions to be evaluated for
        metrics = []    % A list of metrics to be evaluated for
        options         % The workspace simulator options
        graph = []      % The graph representation for the workspace
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = RayWorkspaceSimulator(model,grid,options)
            w@SimulatorBase(model);
            w.grid          = grid;
            w.options       = options;
        end
        
        % Implementation of the run function
        function run(obj, w_conditions, w_metrics)
            if(~obj.options.read_mode)                
                % First determine how big the metrics previously were
                if(isempty(obj.metrics))
                    n_metrics_prev  = 0; 
                    obj.metrics     = w_metrics;
                else
                    n_metrics_prev  = size(obj.metrics,2);
                    obj.metrics = [obj.metrics,w_metrics];
                end
                % Determine how big the conditions previously were
                if(isempty(obj.conditions))
                    n_conditions_prev = 0; 
                    obj.conditions = w_conditions;
                else
                    n_conditions_prev = size(obj.conditions,2);
                    obj.conditions = [obj.conditions,w_conditions];
                end

                % Store the previous workspace
                workspace_prev = obj.workspace;
                % Test if the metrics have infinite limits
                for i = 1:size(w_metrics,2)
                    if((~isempty(w_metrics{i}.metricMax))&&((abs(w_metrics{i}.metricMax)==Inf)||(abs(w_metrics{i}.metricMin)==Inf)))
                        CASPR_log.Print('A metric with infinite limit values cannot be plotted.  To plot please set the metric limit to be finite or filter the workspace after plotting',CASPRLogLevel.WARNING);
                    end
                end

                % Create a cell array for workspace
                % Firstly determine the number of points -- AT THE MOMENT THIS
                % IS ASSUMING UNIFORM GRIDS
                n_grid_points = 0; 
                for i =1:obj.grid.n_dimensions
                    grid_index = true(obj.grid.n_dimensions,1); grid_index(i) = false;
                    n_grid_points = n_grid_points + prod(obj.grid.q_length(grid_index));
                end % THIS NEEDS TO BE TESTED HERE
                obj.workspace = cell(n_grid_points,1);
                workspace_count = 0;
                n_metrics       = length(obj.metrics);
                n_conditions    = length(obj.conditions);
                % Determine translation from workspace_in to current metrics
                % list            

                % Runs over each dimension and construct the rays for that
                % dimension
                % each point
                k = 1;
                for i = 1:obj.grid.n_dimensions
                    grid_index = true(obj.grid.n_dimensions,1); grid_index(i) = false;
                    % Create a subgrid
                    sub_grid = UniformGrid(obj.grid.q_begin(grid_index),obj.grid.q_end(grid_index),obj.grid.delta_q(grid_index),'step_size',obj.grid.q_wrap(grid_index));
                    for j = 1:sub_grid.n_points
                        CASPR_log.Info([sprintf('Workspace ray %d. ',k),sprintf('Completion Percentage: %3.2f',100*k/n_grid_points)]);
                        % Load the current fixed grid coordinates
                        q_fixed = sub_grid.getGridPoint(j);
                        % Construct the workspace ray
                        wr = WorkspaceRay(q_fixed,n_metrics,n_conditions,i,[obj.grid.q_begin(i),obj.grid.q_end(i)]);

                        % For each metric compute the value of the ray
                        for j_m=1:n_metrics
                            %% THIS NEEDS TO BE FILLED IN
                        end
                        % For each condition
                        for j_c=1:n_conditions
                            %% THIS NEEDS TO BE FILLED IN
                            if(j_c < n_conditions_prev)
    %                            if((~isempty(workspace_prev{k}))&&(~isempty(workspace_prev{k}.conditions{j_c})))
    %                                % The metric is at valid value
    %                                wr.addCondition(workspace_prev{i}.conditions{j_c,1},j_c);
    %                            end
                            else
                                % New condition
                                [condition_type, condition_intervals, comp_time] = obj.conditions{j_c}.evaluate(obj.model,wr);
                                if(~isempty(condition_intervals))
                                    wr.addCondition(condition_type,condition_intervals,j_c);
                                end
                            end
                        end
                        % Determine whether to add the condition to the workspace
                        test_conditions = cellfun(@isempty,wr.conditions);
                        % Determine whether to add to workspace
                        if(obj.options.union)
                            entry_condition = (~isempty(obj.metrics)||(sum(test_conditions(:,1))~=n_conditions));
                        else
                            entry_condition = (sum(test_conditions(:,1))==0);
                        end
                        if(entry_condition)
                            % Add the workspace point to the 
                            obj.workspace{k} = wr;
                            workspace_count = workspace_count + 1;
                        end
                        % Note that another ray has been constructed
                        k = k+1;
                    end
                end              
                obj.createWorkspaceGraph();
            end
        end
        
        %       plotRayWorkspace is a temporary function for plotting the computed
        %       ray workspace and verify if the result is reasonable       
        function plotRayWorkspace(obj,plot_axis)   %nsegvar= the division number of interval of variables
            
            numDofs=obj.grid.n_dimensions;
            if numDofs>2
                if(nargin<2)
                    plot_axis=[1,2,3];
                end
                axis1=[];
                axis2=[];
                axis3=[];
                [size_workspace, ~]=size(obj.workspace);
                for it=1:size_workspace
                    if ~isempty(obj.workspace{it})
                        plotflag=0;
                        consvar=zeros(1,numDofs);
                        consvar(1:obj.workspace{it}.free_variable_index-1)=obj.workspace{it}.fixed_variables(1:obj.workspace{it}.free_variable_index-1,1);
                        consvar(obj.workspace{it}.free_variable_index)=0;
                        consvar(obj.workspace{it}.free_variable_index+1:numDofs)=obj.workspace{it}.fixed_variables(obj.workspace{it}.free_variable_index:numDofs-1,1);
                        if obj.workspace{it}.free_variable_index==plot_axis(1)
                            axis1=obj.workspace{it}.conditions{2};
%                             axis1=obj.workspace{it}.free_variable_range;
                            axis2=(consvar(plot_axis(2))*ones(1,2));
                            axis3=(consvar(plot_axis(3))*ones(1,2));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(2)
                            axis1=(consvar(plot_axis(1))*ones(1,2));
                            axis2=obj.workspace{it}.conditions{2};
%                             axis2=obj.workspace{it}.free_variable_range;
                            axis3=(consvar(plot_axis(3))*ones(1,2));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(3)
                            axis1=(consvar(plot_axis(1))*ones(1,2));
                            axis2=(consvar(plot_axis(2))*ones(1,2));
                            axis3=obj.workspace{it}.conditions{2};
%                             axis3=obj.workspace{it}.free_variable_range;
                            plotflag=1;
                        end
                        if plotflag==1
                            plot3(axis1,axis2,axis3,'k','LineWidth',1);
                            axis1=strcat('axis_',int2str(plot_axis(1)));
                            axis2=strcat('axis_',int2str(plot_axis(2)));
                            axis3=strcat('axis_',int2str(plot_axis(3)));
                            xlabel(axis1)
                            ylabel(axis2)
                            zlabel(axis3)
                            hold on
                        end
                    end
                end
            else
                if(nargin<2)
                    plot_axis=[1,2];
                end
                axis1=[];
                axis2=[];
                [size_workspace, ~]=size(obj.workspace);
                for it=1:size_workspace                  
                    if ~isempty(obj.workspace{it})
                        plotflag=0;
                        consvar=zeros(1,numDofs);
                        consvar(1:obj.workspace{it}.free_variable_index-1)=obj.workspace{it}.fixed_variables(1:obj.workspace{it}.free_variable_index-1,1);
                        consvar(obj.workspace{it}.free_variable_index)=0;
                        consvar(obj.workspace{it}.free_variable_index+1:numDofs)=obj.workspace{it}.fixed_variables(obj.workspace{it}.free_variable_index:numDofs-1,1);
                        if obj.workspace{it}.free_variable_index==plot_axis(1)
                            axis1=obj.workspace{it}.free_variable_range;
                            axis2=(consvar(plot_axis(2))*ones(1,2));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(2)                         
                            axis1=(consvar(plot_axis(1))*ones(1,2));
                            axis2=obj.workspace{it}.free_variable_range;
                            plotflag=1;
                        end
                        if plotflag==1
                            plot(axis1,axis2,'k','LineWidth',1);
                            axis1=strcat('axis_',int2str(plot_axis(1)));
                            axis2=strcat('axis_',int2str(plot_axis(2)));
                            xlabel(axis1)
                            ylabel(axis2)
                            hold on
                        end
                    end
                end
            end
        end
    end
    
    methods(Access = private)
        function createWorkspaceGraph(obj)
            node_length = length(obj.workspace);
            obj.graph = zeros(node_length,node_length);
            for i = 1:node_length
                if(~isempty(obj.workspace{i}))
                    obj.graph(i,i) = 1;
                    for j = i+1:node_length
                        if(~isempty(obj.workspace{j}))
                            obj.graph(i,j) = obj.workspace{i}.intersect(obj.workspace{j});
                            obj.graph(j,i) = obj.graph(i,j);
                        end
                    end
                end
            end
        end
    end

end
