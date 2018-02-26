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
            cmap = 0.5*ones(257,3);
            cmap(1:128,:) = winter(128);
            cmap(130:257,:) = flipud(autumn(128));
            for i = 1:obj.grid.n_dimensions+1+metric_flag
                figure;
                G.Edges.EdgeColours = obj.graph_rep(:,2+i); % INTERSECT NEEDS TO BE CHANGED TO INCLUDE THE POINT ITSELF
                p = plot(G);
                layout(p,'auto')  
                edge_range = (max(G.Edges.EdgeColours)-min(G.Edges.EdgeColours));
                zero_colour = min(G.Edges.EdgeColours) - edge_range/256;
                if(i <= obj.grid.n_dimensions)
                    centroid_list = mean(obj.node_list(:,2:3),2);
                    max_centroid_list = max(centroid_list(obj.node_list(:,3+obj.grid.n_dimensions)==i));
                    min_centroid_list = min(centroid_list(obj.node_list(:,3+obj.grid.n_dimensions)==i));
                    G.Nodes.NodeColours = zeros(G.numnodes,1);
                    for j = 1:G.numnodes
                        if(obj.node_list(j,3+obj.grid.n_dimensions)==i)
                            G.Nodes.NodeColours(j) = -edge_range/(max_centroid_list-min_centroid_list)*(centroid_list(j) - min_centroid_list) + min(G.Edges.EdgeColours) - edge_range/128;
                        else
                            G.Nodes.NodeColours(j) = zero_colour;
                        end
                    end
                    p.NodeCData = G.Nodes.NodeColours;
                end
                p.EdgeCData = G.Edges.EdgeColours;
                p.LineWidth = 2;
                set(gca,'XTick',[]);set(gca,'YTick',[]);
                colormap(cmap)
                colorbar
            end
        end
        
%         function plotGraphAdjacency(obj)
%             % Determine the maximum number of rays
%             number_points = length(obj.workspace);
%             max_number_rays = 0;
%             for i = 1:number_points
%                 if(~isempty(obj.workspace{i}))
%                     number_rays = size(obj.workspace{i}.conditions{1,2},1);
%                     if(number_rays > max_number_rays)
%                         max_number_rays = number_rays;
%                     end
%                 end
%             end
%             number_nodes = max_number_rays*number_points;
%             adjacency_matrix = zeros(number_nodes,number_nodes);
%             % FIGURE OUT HOW TO BREAK DOWN
%             number_points_1 = 1;
%             for i = 1:number_points
%                 if(~isempty(obj.workspace{i}))
%                     adjacency_matrix(i,i) = 1;
%                     for j = i+1:number_points
%                         if(~isempty(obj.workspace{j}))
%                             % Check if there is an intersection
%                             adjacency_matrix(i,j) = obj.workspace{i}.intersect(obj.workspace{i}.conditions{1,2},obj.workspace{j},obj.workspace{j}.conditions{1,2});
%                         end
%                     end
%                 end
%                 number_points_1 = number_points_1 + 1;
%             end
%             
%             % REMOVE THE HARDCODING
%             % DETERMINE HOW TO LABEL
%             
%             % Perform reordering for slice plot construction
%             % Determine the number of nodes per dimension
%             number_nodes_per_dimension = number_nodes/obj.grid.n_dimensions;
%             number_point = 25;
% %             adjacency_matrix(1:number_nodes_per_dimension,number_nodes_per_dimension+1:2*number_nodes_per_dimension) = adjacency_matrix([1:25:number_nodes_per_dimension,2:25:number_nodes_per_dimension,3:25:number_nodes_per_dimension,4:25:number_nodes_per_dimension,5:25:number_nodes_per_dimension,6:25:number_nodes_per_dimension,7:25:number_nodes_per_dimension,8:25:number_nodes_per_dimension,9:25:number_nodes_per_dimension,10:25:number_nodes_per_dimension,11:25:number_nodes_per_dimension,12:25:number_nodes_per_dimension,13:25:number_nodes_per_dimension,14:25:number_nodes_per_dimension,15:25:number_nodes_per_dimension,16:25:number_nodes_per_dimension,17:25:number_nodes_per_dimension,18:25:number_nodes_per_dimension,19:25:number_nodes_per_dimension,20:25:number_nodes_per_dimension,21:25:number_nodes_per_dimension,22:25:number_nodes_per_dimension,23:25:number_nodes_per_dimension,24:25:number_nodes_per_dimension,25:25:number_nodes_per_dimension],...
% %                 [number_nodes_per_dimension+1:25:2*number_nodes_per_dimension,number_nodes_per_dimension+2:25:2*number_nodes_per_dimension,number_nodes_per_dimension+3:25:2*number_nodes_per_dimension,number_nodes_per_dimension+4:25:2*number_nodes_per_dimension,number_nodes_per_dimension+5:25:2*number_nodes_per_dimension,number_nodes_per_dimension+6:25:2*number_nodes_per_dimension,number_nodes_per_dimension+7:25:2*number_nodes_per_dimension,number_nodes_per_dimension+8:25:2*number_nodes_per_dimension,number_nodes_per_dimension+9:25:2*number_nodes_per_dimension,number_nodes_per_dimension+10:25:2*number_nodes_per_dimension,number_nodes_per_dimension+11:25:2*number_nodes_per_dimension,number_nodes_per_dimension+12:25:2*number_nodes_per_dimension,number_nodes_per_dimension+13:25:2*number_nodes_per_dimension,number_nodes_per_dimension+14:25:2*number_nodes_per_dimension,number_nodes_per_dimension+15:25:2*number_nodes_per_dimension,number_nodes_per_dimension+16:25:2*number_nodes_per_dimension,number_nodes_per_dimension+17:25:2*number_nodes_per_dimension,number_nodes_per_dimension+18:25:2*number_nodes_per_dimension,number_nodes_per_dimension+19:25:2*number_nodes_per_dimension,number_nodes_per_dimension+20:25:2*number_nodes_per_dimension,number_nodes_per_dimension+21:25:2*number_nodes_per_dimension,number_nodes_per_dimension+22:25:2*number_nodes_per_dimension,number_nodes_per_dimension+23:25:2*number_nodes_per_dimension,number_nodes_per_dimension+24:25:2*number_nodes_per_dimension,number_nodes_per_dimension+25:25:2*number_nodes_per_dimension]);
% %             adjacency_matrix(1:number_nodes_per_dimension,2*number_nodes_per_dimension+1:3*number_nodes_per_dimension) = adjacency_matrix(1:number_nodes_per_dimension,...
% %                 [2*number_nodes_per_dimension+1:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+2:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+3:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+4:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+5:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+6:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+7:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+8:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+9:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+10:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+11:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+12:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+13:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+14:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+15:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+16:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+17:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+18:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+19:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+20:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+21:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+22:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+23:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+24:25:3*number_nodes_per_dimension,2*number_nodes_per_dimension+25:25:3*number_nodes_per_dimension]);
%             adjacency_matrix(1:number_nodes_per_dimension,number_nodes_per_dimension+1:2*number_nodes_per_dimension) = adjacency_matrix([1:5:number_nodes_per_dimension,2:5:number_nodes_per_dimension,3:5:number_nodes_per_dimension,4:5:number_nodes_per_dimension,5:5:number_nodes_per_dimension],...
%                 [number_nodes_per_dimension+1:5:2*number_nodes_per_dimension,number_nodes_per_dimension+2:5:2*number_nodes_per_dimension,number_nodes_per_dimension+3:5:2*number_nodes_per_dimension,number_nodes_per_dimension+4:5:2*number_nodes_per_dimension,number_nodes_per_dimension+5:5:2*number_nodes_per_dimension]);
%             adjacency_matrix(1:number_nodes_per_dimension,2*number_nodes_per_dimension+1:3*number_nodes_per_dimension) = adjacency_matrix(1:number_nodes_per_dimension,...
%                 [2*number_nodes_per_dimension+1:5:3*number_nodes_per_dimension,2*number_nodes_per_dimension+2:5:3*number_nodes_per_dimension,2*number_nodes_per_dimension+3:5:3*number_nodes_per_dimension,2*number_nodes_per_dimension+4:5:3*number_nodes_per_dimension,2*number_nodes_per_dimension+5:5:3*number_nodes_per_dimension]);
%             % Determine a reordering approach - start with 3D and extend
%             cmap = hot(5);
%             subplot(3,3,1); 
%             for i = 1:25
%                 if(adjacency_matrix(i,i))
%                     plot(i,i,'.','color',[0,0,0]); hold on
%                 end
%             end
%             axis([1,25,1,25]);
%             xlabel('q_1 ray #'); set(gca,'xaxisLocation','top'); ylabel('q_1 ray #');
%             set(gca,'Ydir','reverse');
% %             imagesc(5-5*adjacency_matrix(1:number_nodes_per_dimension,1:number_nodes_per_dimension)); colormap(fliplr(hot)); xlabel('q_1 ray #'); set(gca,'xaxisLocation','top'); ylabel('q_1 ray #'); 
%             scaling_matrix = blkdiag(eye(5),2*eye(5),3*eye(5),4*eye(5),5*eye(5));
%             subplot(3,3,2); 
%             for i = 1:25
%                 for j = 1:25
%                     if(adjacency_matrix(i,25+j))
%                         plot(j,i,'.','color',cmap(4-floor(j/5),:)); hold on
%                     end
%                 end
%             end
%             axis([1,25,1,25]);
%             xlabel('q_2 ray #'); set(gca,'xaxisLocation','top'); set(gca,'yTick',[]); 
%             set(gca,'Ydir','reverse');
%             
% %             imagesc(5-scaling_matrix*adjacency_matrix(1:number_nodes_per_dimension,number_nodes_per_dimension+1:2*number_nodes_per_dimension)); colormap(hot); xlabel('q_2 ray #'); set(gca,'xaxisLocation','top'); set(gca,'yTick',[]); 
%             subplot(3,3,3); 
% %             imagesc(5-scaling_matrix*adjacency_matrix(1:number_nodes_per_dimension,2*number_nodes_per_dimension+1:3*number_nodes_per_dimension)); colormap(hot); xlabel('q_3 ray #'); set(gca,'xaxisLocation','top'); set(gca,'yTick',[]); 
%             for i = 1:25
%                 for j = 1:25
%                     if(adjacency_matrix(i,50+j))
%                         plot(j,i,'.','color',cmap(4-floor(j/5),:)); hold on
%                     end
%                 end
%             end
%             axis([1,25,1,25]);
%             xlabel('q_3 ray #'); set(gca,'xaxisLocation','top'); set(gca,'yTick',[]); 
%             set(gca,'Ydir','reverse');
%             subplot(3,3,5); 
%             for i = 1:25
%                 if(adjacency_matrix(25+i,25+i))
%                     plot(i,i,'.','color',[0,0,0]); hold on
%                 end
%             end
%             axis([1,25,1,25]);
%             set(gca,'xTick',[]); ylabel('q_2 ray #');
%             set(gca,'Ydir','reverse');
% %             imagesc(5-5*adjacency_matrix(number_nodes_per_dimension+1:2*number_nodes_per_dimension,number_nodes_per_dimension+1:2*number_nodes_per_dimension)); colormap(hot); set(gca,'xTick',[]); ylabel('q_2 ray #');
%             subplot(3,3,6); 
%             for i = 1:25
%                 for j = 1:25
%                     if(adjacency_matrix(25+i,50+j))
%                         plot(j,i,'.','color',cmap(4-floor(j/5),:)); hold on
%                     end
%                 end
%             end
%             axis([1,25,1,25]);
%             set(gca,'xTick',[]); set(gca,'yTick',[]); 
%             set(gca,'Ydir','reverse');
% %             imagesc(5-scaling_matrix*adjacency_matrix(number_nodes_per_dimension+1:2*number_nodes_per_dimension,2*number_nodes_per_dimension+1:3*number_nodes_per_dimension)); colormap(hot); set(gca,'xTick',[]); set(gca,'yTick',[]); 
%             subplot(3,3,9); 
%             for i = 1:25
%                 if(adjacency_matrix(50+i,50+i))
%                     plot(i,i,'.','color',[0,0,0]); hold on
%                 end
%             end
%             axis([1,25,1,25]);
%             set(gca,'xTick',[]); ylabel('q_3 ray #');
%             set(gca,'Ydir','reverse');
% %             imagesc(5-5*adjacency_matrix(2*number_nodes_per_dimension+1:3*number_nodes_per_dimension,2*number_nodes_per_dimension+1:3*number_nodes_per_dimension)); colormap(hot); set(gca,'xTick',[]); ylabel('q_3 ray #');
% %             for i = 1:25
% %                 subplot(1,25,i); imagesc(adjacency_matrix((i-1)*25+1:(i-1)*25+25,number_nodes_per_dimension+(i-1)*25+1:number_nodes_per_dimension+(i-1)*25+25)); colormap(gray);
% %             end
%         end
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
                        obj.graph_rep(number_intersects,1:2+obj.grid.n_dimensions+1) = [i,j,intersection_point',min([min(abs((intersection_point - obj.grid.q_begin)./(obj.grid.q_end - obj.grid.q_begin))),min(abs((intersection_point - obj.grid.q_end)./(obj.grid.q_end - obj.grid.q_begin)))])];
                        if(metric_flag)
                            obj.model.update(intersection_point,zeros(obj.grid.n_dimensions,1),zeros(obj.grid.n_dimensions,1),zeros(obj.grid.n_dimensions,1));
%                             obj.metrics{1}.evaluate()
                            [~,obj.graph_rep(number_intersects,2+obj.grid.n_dimensions+2),~] = obj.metrics{1}.evaluate(obj.model,[]);
                        end
                    end
                end
            end
            obj.graph_rep = obj.graph_rep(1:number_intersects,:);
        end
    end
end
