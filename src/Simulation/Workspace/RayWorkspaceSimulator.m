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
        graph_rep = []  % The graph representation for the workspace
        node_list = []  % A list of all nodes
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
                if((nargin == 2) || isempty(w_metrics))
                    obj.createWorkspaceGraph(w_conditions{1},[]);
                else
                    obj.createWorkspaceGraph(w_conditions{1},w_metrics{1});
                end
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
                            axis2=(consvar(plot_axis(2))*ones(1,2));
                            axis3=(consvar(plot_axis(3))*ones(1,2));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(2)
                            axis1=(consvar(plot_axis(1))*ones(1,2));
                            axis2=obj.workspace{it}.conditions{2};
                            axis3=(consvar(plot_axis(3))*ones(1,2));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(3)
                            axis1=(consvar(plot_axis(1))*ones(1,2));
                            axis2=(consvar(plot_axis(2))*ones(1,2));
                            axis3=obj.workspace{it}.conditions{2};
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
                            axis1=obj.workspace{it}.conditions{2};
                            axis2=(consvar(plot_axis(2))*ones(1,2));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(2)                         
                            axis1=(consvar(plot_axis(1))*ones(1,2));
                            axis2=obj.workspace{it}.conditions{2};
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
        
        % A function for plotting a graph
        function plotGraph(obj)
            % Shrink the graph to remove all nodes that are empty
            G = graph(obj.graph_rep(:,1),obj.graph_rep(:,2));
            if(isempty(obj.metrics))
                metric_flag = 0;
            else
                metric_flag = 1;
            end
            cmap = 0.7*ones(257,3);
            cmap(1:128,:) = winter(128);
            cmap(130:257,:) = flipud(autumn(128));
            for i = 1:obj.grid.n_dimensions+1+metric_flag
                figure;
                G.Edges.EdgeColours = obj.graph_rep(:,2+i); % INTERSECT NEEDS TO BE CHANGED TO INCLUDE THE POINT ITSELF
                ax1 = axes;
                p = plot(G,'MarkerSize',2);
                layout(p,'auto')  
                edge_range = (max(G.Edges.EdgeColours)-min(G.Edges.EdgeColours));
                zero_colour = min(G.Edges.EdgeColours) - edge_range/256;
                if(i <= obj.grid.n_dimensions)
                    centroid_list = mean(obj.node_list(:,2:3),2);
                    max_list = max(centroid_list(obj.node_list(:,3+obj.grid.n_dimensions)==i));
                    min_list = min(centroid_list(obj.node_list(:,3+obj.grid.n_dimensions)==i));
                    G.Nodes.NodeColours = zeros(G.numnodes,1);
                    for j = 1:G.numnodes
                        if(obj.node_list(j,3+obj.grid.n_dimensions)==i)
                            G.Nodes.NodeColours(j) = -edge_range/(max_list-min_list)*(centroid_list(j) - min_list) + min(G.Edges.EdgeColours) - edge_range/128;
                        else
                            G.Nodes.NodeColours(j) = zero_colour;
                        end
                    end
                    p.NodeCData = G.Nodes.NodeColours;
                else
                    ray_distance_list = zeros(G.numnodes,1);
                    for j = 1:G.numnodes
                        ray_distance_list(j) =  (obj.node_list(j,3) - obj.node_list(j,2))/(obj.grid.q_end(obj.node_list(j,3+obj.grid.n_dimensions)) - obj.grid.q_begin(obj.node_list(j,3+obj.grid.n_dimensions)));
                    end
                    max_list = max(ray_distance_list);
                    min_list = min(ray_distance_list);
                    G.Nodes.NodeColours = zeros(G.numnodes,1);
                    for j = 1:G.numnodes
                        G.Nodes.NodeColours(j) = -edge_range/(max_list-min_list)*(ray_distance_list(j) - min_list) + min(G.Edges.EdgeColours) - edge_range/128;
                    end
                    p.NodeCData = G.Nodes.NodeColours;
                end
                p.EdgeCData = G.Edges.EdgeColours;
                p.LineWidth = 2;
                set(gca,'XTick',[]);set(gca,'YTick',[]);
                ax2 = axes;
                linkaxes([ax1,ax2]);
                %% Hide the top axes
                ax2.Visible = 'off';
                ax2.XTick = [];
                ax2.YTick = [];
                colormap(ax1,cmap);
                colormap(ax2,winter(128));
                %                 colorbar
                set([ax1,ax2],'Position',[.15 .11 .685 .815]);
                cb1 = colorbar(ax1,'Position',[.07 .11 .0675 .815]);
                set(cb1,'Limits',[min(G.Edges.EdgeColours),max(G.Edges.EdgeColours)])
                cb2 = colorbar(ax2,'Position',[.85 .11 .0675 .815]);
                caxis(ax2,[min_list,max_list]);
            end
        end
    end
    
    methods(Access = private)
        function createWorkspaceGraph(obj,condition,metric)
            if(isempty(metric))
                metric_flag = 0;
            else
                metric_flag = 1;
            end
                
            % For the moment this will be written as if there was only one
            % condition I will subsequently modify
            number_rays = length(obj.workspace);
            % Create list of nodes
            obj.node_list = zeros(10*number_rays,3+obj.grid.n_dimensions); % THIS DATA TYPE WILL CHANGE LATER TO BE A STRUCT OR CLASS
            number_node = 0;
            % For each ray
            for i = 1:number_rays
                % Determine the number of the condition - NOTE THIS COULD
                % POSSIBLY MOVE UP
                if(~isempty(obj.workspace{i}))
                    n_constraints = size(obj.workspace{i}.conditions,1);
                    intervals = [];
                    for j = 1:n_constraints
                        if(condition.type == obj.workspace{i}.conditions{j,1})
                            intervals = obj.workspace{i}.conditions{j,2};
                            break;
                        end
                    end
                    for j = 1:size(intervals,1)
                        number_node = number_node + 1;
                        obj.node_list(number_node,:) = [i,intervals(j,:),obj.workspace{i}.fixed_variables',obj.workspace{i}.free_variable_index];
                    end
                end
            end                
            % Resize to the correct size
            obj.node_list = obj.node_list(1:number_node,:); 
            
            % Initialise an adjacency list
            obj.graph_rep = zeros(number_node*number_node,2+obj.grid.n_dimensions+1+metric_flag);
            number_intersects = 0;
            for i = 1:number_node
                i
                for j = i+1:number_node
                    [is_intersected,intersection_point] = obj.workspace{obj.node_list(i,1)}.intersect(obj.node_list(i,2:3),obj.workspace{obj.node_list(j,1)},obj.node_list(j,2:3));
                    if(is_intersected)
                        number_intersects = number_intersects + 1;
                        min_dist = min([abs(intersection_point(obj.node_list(i,3+obj.grid.n_dimensions))-obj.node_list(i,2)),abs(intersection_point(obj.node_list(i,3+obj.grid.n_dimensions))-obj.node_list(i,3)),abs(intersection_point(obj.node_list(j,3+obj.grid.n_dimensions))-obj.node_list(j,2)),abs(intersection_point(obj.node_list(j,3+obj.grid.n_dimensions))-obj.node_list(j,3))]);
%                         % Go through all of the nodes to determine if there
%                         % is another node that is closer and intersects
%                         % with the point of intersection
%                         for k = 1:number_node
%                             selection_vector = true(obj.grid.n_dimensions,1); selection_vector(obj.node_list(k,3+obj.grid.n_dimensions)) = false;
%                             % Condition to ensure that it is suitable
%                             if((k~=i)&&(k~=j)&&(sum(obj.node_list(k,4:3+obj.grid.n_dimensions-1)'==intersection_point(selection_vector))==obj.grid.n_dimensions-1))
%                                 % Compute distance
%                                 k_dist = min([abs(intersection_point(obj.node_list(k,3+obj.grid.n_dimensions))-obj.node_list(k,2)),abs(intersection_point(obj.node_list(k,3+obj.grid.n_dimensions))-obj.node_list(k,3))]);
%                                 % If smaller update
%                                 if(k_dist <= min_dist)
%                                     min_dist = k_dist;
%                                 end
%                             end
%                         end
                        obj.graph_rep(number_intersects,1:2+obj.grid.n_dimensions+1) = [i,j,intersection_point',min_dist];
                        if(metric_flag)
                            obj.model.update(intersection_point,zeros(obj.grid.n_dimensions,1),zeros(obj.grid.n_dimensions,1),zeros(obj.grid.n_dimensions,1));
                            [~,obj.graph_rep(number_intersects,2+obj.grid.n_dimensions+2),~] = obj.metrics{1}.evaluate(obj.model,[]);
                        end
                    end
                end
            end
            obj.graph_rep = obj.graph_rep(1:number_intersects,:);
        end
    end
end
