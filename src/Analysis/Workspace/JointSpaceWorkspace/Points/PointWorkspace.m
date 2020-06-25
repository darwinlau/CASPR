% A container class to hold workspace result for a point pose
%
% Author        : Darwin LAU, Paul Cheng
% Created       : 2019
% Description    : This class contains the known information obtained
% through workspace analysis for all poses
classdef PointWorkspace < handle
    properties (SetAccess = protected)
        model
        grid                        % Grid object for brute force workspace
        conditions                  % Cell array of workspace conditions
        metrics                     % Cell array of workspace metrics
    end
    
    properties
        poses                       % Cell array
        graph_rep = []              % The graph representation for the workspace
        node_list = []              % A list of all nodes
    end
    properties  (Hidden = true)
        layer_ws_figure                       % Cell array
    end
    
    methods
        function pw = PointWorkspace(model, conditions, metrics, grid)
            pw.model = model;
            pw.grid = grid;
            pw.conditions = conditions;
            pw.metrics = metrics;
            pw.poses = cell(grid.n_points,1);
        end
        
        % This function generates a graph workspace representation based on
        % a set of conditions and metrics
        function createWorkspaceGraph(obj, conditions, metrics, w_connectivity)
            if(~isempty(metrics))
                metric_types = input_conversion(metrics');
                pose_metric_types = input_conversion(obj.poses{1}.metrics);
                
                if(~all(ismember(metric_types,pose_metric_types)))
                    CASPR_log.Error('Input metric(s) NOT exist in the workspace metric(s)')
                end
            end
            obj.node_list = create_node_list(obj, conditions);
            obj.graph_rep =  create_graph_rep(obj, w_connectivity, size(metrics,2));
        end
        
        % Function to plot the workspace in 2D or 3D. For robots with more
        % than 3 DoFs, values of the fixed DoFs need to be provided to
        % lower its dimension to 2D or 3D.
        %
        % Inputs:
        %   - dofs_to_plot: The array of the joint pose (q) indices to plot
        %       in 2-D or 3-D (in the order of XYZ)
        %   - conditions_ind: The array of the indices of the workspace
        %       conditions to be plotted
        %   - metrics_ind: The array of the indices of the workspace
        %       metrics to be plotted
        %   - fixed_var_val: The array of the values for the fixed
        %       variables when the DoF of the robot is greater than the
        %       dimension of the plot. Note that the dimension of the array
        %       must the same as the DoF of the robot, the fixed values for
        %       the DoFs to be plotted will be ignored.
        function w_handles = plotWorkspace(obj, dofs_to_plot, conditions_ind, metrics_ind, fixed_var_val, is_plot_current)
            digit_tolerance = 4;
            
            % Start with a set of checking conditions
            CASPR_log.Assert(length(dofs_to_plot) == 2 || length(dofs_to_plot) == 3, 'Number of DoFs to plot must be 2 or 3, otherwise it cannot be plotted.');
            CASPR_log.Assert(length(dofs_to_plot) <= obj.model.numDofs, 'The number of DoFs to plot is more than the degrees of freedom available.');
            if (length(dofs_to_plot) > obj.model.numDofs)
                CASPR_log.Assert(nargin > 4, 'Must input ''fixed_var_val'' if the DoFs to plot does not cover all DoFs.');
                CASPR_log.Assert(length(fixed_var_val) == obj.model.numDofs, 'Input ''fixed_var_val'' must have the same dimension as the DoFs of the robot.');
            end
            
            % If no argument or conditions_ind is empty, then all
            % conditions will be plotted.
            % If conditions_ind == [0], then the metrics for any points
            % within the PointWorkspace (hence the union of the workspace
            % conditions) will be plotted.
            if nargin < 3 || isempty(conditions_ind)
                conditions_ind = 1:length(obj.conditions);
            elseif conditions_ind == 0
                conditions_ind = [];
            end
            % If no argument or metrics_ind is empty, then all metrics will
            % be plotted.
            % If metrics_ind == [0], then no metrics will be plotted even
            % any exist
            if nargin < 4 || isempty(metrics_ind)
                metrics_ind = 1:length(obj.metrics);
            elseif metrics_ind == 0
                metrics_ind = [];
            end
            if nargin < 5
                fixed_var_val = zeros(1, obj.model.numDofs);
            end
            if nargin < 6
                is_plot_current = false;
            end
            
            CASPR_log.Assert(~isempty(conditions_ind) || ~isempty(metrics_ind), 'There should at least be one metric or condition specified.');
            
            fixed_var_val = reshape(fixed_var_val, [1,obj.model.numDofs]);
            num_metrics = size(metrics_ind,2);
            
            point_color_matrix = 0;
            
            % Filter out the points to plot            
            if ~isempty(conditions_ind)
                filtered_node_list = create_node_list(obj, conditions_ind);
                pose_data = round(filtered_node_list(:,2:end), digit_tolerance);
                % If no conditions then all points should be plotted
            else
                num_poses = size(obj.poses, 1);
                pose_data = zeros(num_poses, obj.model.numDofs);
                filtered_node_list = zeros(num_poses, obj.model.numDofs+1);
                for i = 1:num_poses
                    pose_data(i,:) = round(obj.poses{i}.pose',digit_tolerance);
                    filtered_node_list(i,:) = [i, obj.poses{i}.pose'];
                end
            end
            fixed_var_val = round(fixed_var_val,digit_tolerance);
            pose_data(:, dofs_to_plot) = [];
            fixed_var_val(:, dofs_to_plot) = [];
            matched_poses_indices = find(ismember(pose_data, fixed_var_val, 'rows'));
            points_to_plot = filtered_node_list(matched_poses_indices, 2:end);
            
            % Construct the point colour matrix if there are metrics to
            % plot
            if ~isempty(metrics_ind)
                point_color_matrix = zeros(num_metrics, size(matched_poses_indices,1));
                for i = 1:size(matched_poses_indices,1)
                    for j = 1:num_metrics
                        %metrics_indices = find(ismember(pose_metric_types,metric_types(j)));
                        point_color_matrix(j,i) = obj.poses{filtered_node_list(matched_poses_indices(i),1)}.metrics{metrics_ind(j),2};
                    end
                end
            end
            
            % universal plotting function for 2d/3d
            if ~isempty(matched_poses_indices)
                w_handles = universal_plot(obj, dofs_to_plot, points_to_plot, point_color_matrix, is_plot_current);
            else
                CASPR_log.Warn('There are no points for this workspace to plot');
                cla
                w_handles = [];
            end
        end
        
        % Plotting function to plot 2D/3D (subset of the) workspace with slider
        function f = plotWorkspaceSlider(obj, plot_axis, slide_axis, conditions_ind, metrics, fixed_variables)
            num_metrics = size(metrics,2);
            if num_metrics == 1
                metrics = mat2cell(metrics,1);
            elseif num_metrics == 0
                num_metrics = 1;
                metrics{num_metrics} = {};
            end
            if obj.grid.q_begin(slide_axis) == obj.grid.q_end(slide_axis)
                CASPR_log.Error('No sliding options for this axis')
            end
            if ismember(slide_axis,plot_axis)
                CASPR_log.Error('Sliding axis cannot be the plotting axis');
            end
            layer_indices = 1:obj.grid.q_length(slide_axis);
            q_grid = obj.grid.q_begin(slide_axis) + (layer_indices-1)*obj.grid.delta_q(slide_axis);
            obj.layer_ws_figure = cell(num_metrics,obj.grid.q_length(slide_axis));
            current_q_index = find(round(q_grid - fixed_variables(slide_axis),10) == 0);
            
            if isempty(current_q_index)
                CASPR_log.Error('Initial value not available, choose another value');
            end
                   
            for i = 1:num_metrics
                f(i) = figure;
                f(i).Visible = 'on';
                ax = axes('Parent', f(i), 'position', [0.15 0.2 0.7 0.7]);
                workspace_fig = plotWorkspace(obj, plot_axis, conditions_ind, metrics{i}, fixed_variables, true);
                if isempty(workspace_fig)
                    f(i) = figure(i);
                    figure_date = [];
                else
                    f(i) = workspace_fig;
                    ax_old = gcf;
                    scatter_data = findobj(ax_old.Children,'-property','YData');
                    % save important data from the plot for recalling the plot again, do not need to recalculate the node list
                    figure_date.XData = scatter_data.XData;
                    figure_date.YData = scatter_data.YData;
                    figure_date.ZData = scatter_data.ZData;
                    figure_date.CData = scatter_data.CData;
                    figure_date.Marker = scatter_data.Marker;
                    figure_date.MarkerEdgeColor = scatter_data.MarkerEdgeColor;
                    figure_date.MarkerFaceColor = scatter_data.MarkerFaceColor;
                    figure_date.SizeData = scatter_data.SizeData;
                    figure_date.LineWidth = scatter_data.LineWidth;
                end
               
                q_grid = obj.grid.q_begin(slide_axis) + (layer_indices-1)*obj.grid.delta_q(slide_axis);
                current_fixed_variables = fixed_variables;
                current_q_index = find(round(q_grid - fixed_variables(slide_axis),10) == 0);
                title(['Current q_',num2str(slide_axis),' is ', num2str(current_fixed_variables(slide_axis))]);
                t = annotation('textbox', [0.15, 0.03, 0.1, 0.1], 'String',...
                    ['min = ',num2str(q_grid(1))]);
                t.LineStyle = 'none';
                t = annotation('textbox', [0.7, 0.03, 0.1, 0.1], 'String',...
                    ['max = ',num2str(q_grid(end))]);
                t.LineStyle = 'none';
                
                obj.layer_ws_figure{i,current_q_index} = figure_date;
                
                b(i) = uicontrol('Parent', f(i), 'Position', [80, 10, 400, 15], ...
                    'Style', 'slider', 'value', fixed_variables(slide_axis),...
                    'min', obj.grid.q_begin(slide_axis), 'max', obj.grid.q_end(slide_axis), ...
                    'sliderstep',[1/(obj.grid.q_length(slide_axis)-1),1/(obj.grid.q_length(slide_axis)-1)]);
                current_fixed_variables = fixed_variables;
                
                plot_fig = @reloadFig;
                b(i).Callback = @(es,ed) refreshdata(f(i),plot_fig(obj,i,slide_axis,...
                    plot_axis,conditions_ind, metrics{i}, [current_fixed_variables(1:slide_axis-1);es.Value;current_fixed_variables(slide_axis+1:end)]));
            end
        end
        
        function fig = reloadFig(obj,metric_num,slide_axis,plot_axis,conditions_ind,metrics,var)
            layer_indices = 1:obj.grid.q_length(slide_axis);
            q_grid = obj.grid.q_begin(slide_axis) + (layer_indices-1)*obj.grid.delta_q(slide_axis);
            slider_pos_1 = find(var(slide_axis) <= q_grid);
            slider_pos_2 = find(var(slide_axis) >= q_grid);
            if q_grid(slider_pos_1(1)) - var(slide_axis) > q_grid(slider_pos_2(end)) - var(slide_axis)
                var(slide_axis) = q_grid(slider_pos_1(1));
            else
                var(slide_axis) = q_grid(slider_pos_2(end));
            end
            
            current_fixed_variables = var;
            current_q_index = find(round(q_grid - current_fixed_variables(slide_axis),10) == 0);
            
            if ~isempty(obj.layer_ws_figure{metric_num,current_q_index})
                hold on;
                cla
                dataObjs = obj.layer_ws_figure{metric_num,current_q_index};
                if ~isempty(dataObjs)
                    if isempty(dataObjs.ZData)
                        fig = scatter(dataObjs.XData,dataObjs.YData,dataObjs.SizeData,dataObjs.CData,dataObjs.Marker);
                    else
                        fig = scatter3(dataObjs.XData,dataObjs.YData,dataObjs.SizeData,dataObjs.CData,dataObjs.Marker);
                    end
                end
            else
                hold on;
                cla;
                fig = plotWorkspace(obj,plot_axis,conditions_ind, metrics, var, true);
                ax_old = gcf;
                scatter_data = findobj(ax_old.Children,'-property','YData');
                if ~isempty(scatter_data)
                    % save important data from the plot for recalling the plot again, do not need to recalculate the node list
                    
                    figure_date.XData = scatter_data.XData;
                    figure_date.YData = scatter_data.YData;
                    figure_date.ZData = scatter_data.ZData;
                    figure_date.CData = scatter_data.CData;
                    figure_date.Marker = scatter_data.Marker;
                    figure_date.MarkerEdgeColor = scatter_data.MarkerEdgeColor;
                    figure_date.MarkerFaceColor = scatter_data.MarkerFaceColor;
                    figure_date.SizeData = scatter_data.SizeData;
                    figure_date.LineWidth = scatter_data.LineWidth;
                else
                    figure_date = [];
                end
                obj.layer_ws_figure{metric_num,current_q_index} = figure_date;
            end
            title(['Current q_',num2str(slide_axis),' is ', num2str(current_fixed_variables(slide_axis))]);
        end
        
        % A function for plotting a graph
        function node_graph = plotGraph(obj, conditions, metrics, w_connectivity)
            %Create the graph data depends on the inputs
            if isempty(obj.poses)
                CASPR_log.Error('Empty workspace! No plot available')
            end
            createWorkspaceGraph(obj, conditions, metrics, w_connectivity);
            
            G = graph(obj.graph_rep(:,1),obj.graph_rep(:,2));
            if(isempty(metrics))
                num_metrics = 0;
            else
                num_metrics = size(metrics,2);
            end
            cmap = 0.7*ones(257,3);
            cmap(1:128,:) = winter(128);
            cmap(130:257,:) = flipud(autumn(128));
            number_edges = size(obj.graph_rep,1);
            %define the color map of the graph base on the corresponding
            %information such as the distance and the metrics value
            for i = 1:obj.grid.n_dimensions+num_metrics
                figure;
                edge_colour_matrix = nan(number_edges,1);
                for ii = 1:number_edges
                    if(i <= obj.grid.n_dimensions)
                        if(obj.graph_rep(ii,2+i))
                            edge_colour_matrix(ii) = obj.poses{obj.node_list(obj.graph_rep(ii,1),1)}.pose(i);
                        end
                    else
                        edge_colour_matrix(ii) = obj.poses{obj.node_list(obj.graph_rep(ii,1),1)}.metrics{i-obj.grid.n_dimensions,2};
                    end
                    
                end
                ax1 = axes;
                node_graph(i) = plot(G,'MarkerSize',2);
                %Specify the title
                if(i <= obj.grid.n_dimensions)
                    title(['variable ' num2str(i)]);
                else
                    title(['Metric: '  regexprep(char(obj.poses{i}.metrics{i-obj.grid.n_dimensions,1}.type),'_',' ')]);
                end
                layout(node_graph(i),'auto')
                edge_range = (max(edge_colour_matrix)-min(edge_colour_matrix));
                zero_colour = min(edge_colour_matrix) - edge_range/256;
                for ii = 1:number_edges
                    if(~obj.graph_rep(ii,2+i))
                        edge_colour_matrix(ii) = zero_colour;
                    end
                end
                G.Edges.EdgeColours = edge_colour_matrix; % INTERSECT NEEDS TO BE CHANGED TO INCLUDE THE POINT ITSELF
                if(i <= obj.grid.n_dimensions)
                    max_list = max(obj.node_list(:,1+i));
                    min_list = min(obj.node_list(:,1+i));
                    if(min_list == max_list)
                        if(min_list>0)
                            min_list = 0.99*min_list;
                        elseif(min_list == 0)
                            min_list = -0.001;
                        else
                            min_list = 1.01*min_list;
                        end
                    end
                    G.Nodes.NodeColours = zeros(G.numnodes,1);
                    for j = 1:G.numnodes
                        G.Nodes.NodeColours(j) = -edge_range/(max_list-min_list)*(obj.node_list(j,1+i) - min_list) + min(G.Edges.EdgeColours) - edge_range/127.99;
                    end
                    node_graph(i).NodeCData = G.Nodes.NodeColours;
                else
                    node_graph(i).NodeCData = (zero_colour-0.1)*ones(G.numnodes,1);
                end
                if(i <= obj.grid.n_dimensions)
                    node_graph(i).EdgeCData = G.Edges.EdgeColours;
                    node_graph(i).LineWidth = 2;
                    set(gca,'XTick',[]);set(gca,'YTick',[]);
                    ax2 = axes;
                    linkaxes([ax1,ax2]);
                    % Hide the top axes
                    ax2.Visible = 'off';
                    ax2.XTick = [];
                    ax2.YTick = [];
                    colormap(ax1,cmap);
                    colormap(ax2,flipud(winter(128)));
                    %                 colorbar
                    set([ax1,ax2],'Position',[.15 .11 .685 .815]);
                    cb1 = colorbar(ax1,'Position',[.07 .11 .0675 .815]);
                    if(min(G.Edges.EdgeColours)==max(G.Edges.EdgeColours))
                        set(cb1,'Limits',[min(G.Edges.EdgeColours),max(G.Edges.EdgeColours)+1e-4])
                    else
                        set(cb1,'Limits',[min(G.Edges.EdgeColours),max(G.Edges.EdgeColours)])
                    end
                    cb2 = colorbar(ax2,'Position',[.85 .11 .0675 .815]);
                    caxis(ax2,[min_list,max_list]);
                else
                    node_graph(i).EdgeCData = G.Edges.EdgeColours;
                    node_graph(i).LineWidth = 2;
                    set(gca,'XTick',[]);set(gca,'YTick',[]);
                    set(gca,'Position',[.15 .11 .685 .815]);
                    colormap(cmap(129:257,:));
                    cb1 = colorbar(gca,'Position',[.07 .11 .0675 .815]);
                    if(min(G.Edges.EdgeColours)==max(G.Edges.EdgeColours))
                        set(cb1,'Limits',[min(G.Edges.EdgeColours),max(G.Edges.EdgeColours)+1e-4])
                    else
                        set(cb1,'Limits',[min(G.Edges.EdgeColours),max(G.Edges.EdgeColours)])
                    end
                end
            end
        end
        
        % Filter and creat new workspace based on the boundary of the
        % metric values
        function new_workspace = fliterWorkspaceMetric(obj,metrics,metric_min,metric_max)
            if (~isempty(metric_min) && size(metrics,2) ~= size(metric_min,2)) ||...
                    (~isempty(metric_max) && size(metrics,1) ~= size(metric_max,2))
                CASPR_log.Error('Number of metric boundary does not equal to the number of metrics')
            end
            
            new_workspace =  PointWorkspace(obj.model, obj.grid);
            metric_types = input_conversion(metrics');
            pose_metric_types = input_conversion(obj.poses{1}.metrics);
            
            metric_indices = find(ismember(pose_metric_types,metric_types));
            %For every current existing poses
            for i = 1:size(obj.poses,1)
                %check if the metric boundary condition
                for j = 1:size(metrics,2)
                    if ~isempty(metric_min) && obj.poses{i}.metrics{metric_indices(j),2} >= metric_min(j)
                        check_flag(j) = 1;
                    elseif ~isempty(metric_max) && obj.poses{i}.metrics{metric_indices(j),2} <= metric_max(j)
                        check_flag(j) = 1;
                    else
                        check_flag(j) = 0;
                    end
                end
                
                if ~isempty(check_flag) && all(check_flag)
                    new_workspace.poses{i} = obj.poses{i};
                end
            end
            %remove empty poses
            new_workspace.poses = new_workspace.poses(~cellfun('isempty',new_workspace.poses));
        end
        
    end
    methods (Access=private)
        function w_handles = universal_plot(obj, plot_axis, points_to_plot, point_color_matrix, is_plot_current)
            g = groot;
            
            % By default, if is_plot_current is not supplied, then plot on
            % new figure
            if (nargin < 5)
                is_plot_current = false;
            end
            
            if is_plot_current
                w_handles = g.CurrentFigure;
            % Plot on new figures
            else
                for i = size(point_color_matrix,1):-1:1
                    w_handles(i) = figure;
                end
            end
            
            if size(plot_axis,2) == 2 %plot 2D
                x = points_to_plot(:,plot_axis(1));
                y = points_to_plot(:,plot_axis(2));
                for i = 1:size(point_color_matrix,1)
                    c = point_color_matrix(i,:);
                    
                    if size(unique(c),2) == 1
                        figure(w_handles(i));
                        scatter(x, y, 100, 'k', '.');
                    else
                        figure(w_handles(i));
                        scatter(x, y, 100, c', '.');
                        colorbar;
                    end
                    
                    % plotting title and other stuff, nothing important
                    xlim(1.005*[obj.model.bodyModel.q_min(plot_axis(1)),obj.model.bodyModel.q_max(plot_axis(1))]);
                    ylim(1.005*[obj.model.bodyModel.q_min(plot_axis(2)),obj.model.bodyModel.q_max(plot_axis(2))]);
                    xlabel(sprintf('q_%d', plot_axis(1)));
                    ylabel(sprintf('q_%d', plot_axis(2)));
                end
            elseif size(plot_axis,2) == 3 %Plot 3D
                x = points_to_plot(:,plot_axis(1));
                y = points_to_plot(:,plot_axis(2));
                z =  points_to_plot(:,plot_axis(3));
                
                for i = 1:size(point_color_matrix,1)
                    c = point_color_matrix(i,:);
                    if size(unique(c),2) ==1
                        figure(w_handles(i));
                        scatter3(x, y, z, 100, 'k', '.');
                    else
                        figure(w_handles(i));
                        scatter3(x, y, z, 100, c', '.');
                        colorbar;
                    end
                    
                    % plotting title and other stuff, nothing important
                    xlim(1.005*[obj.model.bodyModel.q_min(plot_axis(1)),obj.model.bodyModel.q_max(plot_axis(1))]);
                    ylim(1.005*[obj.model.bodyModel.q_min(plot_axis(2)),obj.model.bodyModel.q_max(plot_axis(2))]);
                    zlim(1.005*[obj.model.bodyModel.q_min(plot_axis(3)),obj.model.bodyModel.q_max(plot_axis(3))]);
                    
                    title('3-D Workspace Plot');
                    xlabel(sprintf('q_%d', plot_axis(1)));
                    ylabel(sprintf('q_%d', plot_axis(2)));
                    zlabel(sprintf('q_%d', plot_axis(3)));
                end
            else
                CASPR_log.Error('Only 2D/3D plot is available. Try slider plot if more than 3 axis')
            end
        end
        
        % function to create the node_list variable
        function node_list = create_node_list(obj, conditions_ind)
            pose_data_all = obj.poses;
            number_points = length(pose_data_all);
            %condition_indices = 1:length(obj.conditions);
            % Create list of nodes
            node_list = zeros(number_points,1+obj.grid.n_dimensions);
            number_node = 0;
            % For each point
            for i = 1:number_points
                % Find out the poses that fulfill the condition(s)
                pose_data = pose_data_all{i};
                if(~isempty(pose_data))
                    if ~isempty(pose_data.conditions)
                        if (all(ismember(conditions_ind, pose_data.conditionsIndices)))
                            number_node = number_node + 1;
                            node_list(number_node,:) = [i, pose_data.pose'];
                        end
%                     else
%                         CASPR_log.Error('No available workspace conditions');
                    end
                end
            end
            % Resize to the correct size
            node_list = node_list(1:number_node, :);
        end
        
        % function to create the graph_rep variable
        function graph_rep = create_graph_rep(obj, w_connectivity,num_metrics)
            number_node = size(obj.node_list,1);
            % Computation for the maximum number of edges
            max_edges = number_node*max(obj.grid.q_length)*obj.grid.n_dimensions;
            
            % Initialise an adjacency list
            graph_rep = zeros(max_edges,2+obj.grid.n_dimensions);
            number_intersects = 0;
            for i = 1:number_node
                for j = i+1:number_node
                    % Check for connectivity
                    [~,is_connected,~] = w_connectivity.evaluate(obj.model,obj.poses{obj.node_list(i,1)},obj.poses{obj.node_list(j,1)});
                    if(is_connected)
                        number_intersects = number_intersects+1;
                        graph_rep(number_intersects,1) = i;
                        graph_rep(number_intersects,2) = j;
                        common_indices = obj.poses{obj.node_list(i,1)}.pose == obj.poses{obj.node_list(j,1)}.pose;
                        graph_rep(number_intersects,3:2++obj.grid.n_dimensions) = common_indices';
                        % FOR THE MOMENT NO METRICS AND NO OTHER STUFF
                    end
                end
            end
            graph_rep = graph_rep(1:number_intersects,:);
            graph_rep  = [graph_rep,ones(size(graph_rep,1),num_metrics)];%Add metrics connectivity
        end
    end
end
