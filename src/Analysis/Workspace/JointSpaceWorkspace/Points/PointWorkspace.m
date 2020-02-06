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
    end
    
    properties
        poses                       % Cell array
        graph_rep = []              % The graph representation for the workspace
        node_list = []              % A list of all nodes
    end
    
    methods
        function pw = PointWorkspace(model, grid)
            pw.model = model;
            pw.grid = grid;
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
        
        % Plotting function to plot 2D/3D (subset of the) workspace plot
        function c_workspace = plotWorkspace(obj,plot_axis,conditions, metrics, fixed_variables)
            
            digit_tolerance = 4;
            
            num_metrics = size(metrics,2);
            if ~isempty(conditions) && isempty(metrics)
                
                filted_node_list = create_node_list(obj, conditions);
                
                pose_data = round(filted_node_list(:,2:end),digit_tolerance);
                fixed_variables = round(fixed_variables,digit_tolerance);
                pose_data(:,[plot_axis]) = [];
                fixed_variables(:,[plot_axis]) = [];
                
                matched_poses_indices = find(ismember(pose_data,fixed_variables,'rows'));
                
                points_to_plot = filted_node_list(matched_poses_indices,2:end);
                point_color_matrix = 0;
                
            elseif ~isempty(conditions) && ~isempty(metrics)
                
                filted_node_list = create_node_list(obj, conditions);
                metric_types = input_conversion(metrics');
                pose_metric_types = input_conversion(obj.poses{1}.metrics);
                
                if(all(ismember(metric_types,pose_metric_types)))
                    pose_data = round(filted_node_list(:,2:end),digit_tolerance);
                    fixed_variables = round(fixed_variables,digit_tolerance);
                    pose_data(:,[plot_axis]) = [];
                    fixed_variables(:,[plot_axis]) = [];
                    
                    matched_poses_indices = find(ismember(pose_data,fixed_variables,'rows'));
                    
                    points_to_plot = filted_node_list(matched_poses_indices,2:end);
                    
                    for i = 1:size(matched_poses_indices,1)
                        for j = 1:num_metrics
                            metrics_indices = find(ismember(pose_metric_types,metric_types(j)));
                            point_color_matrix(j,i) = cell2mat(obj.poses{filted_node_list(matched_poses_indices(i,1))}.metrics(metrics_indices,2));
                        end
                    end
                else
                    CASPR_log.Error('Input metric(s) NOT exist in the workspace metric(s)')
                end
                
            elseif ~isempty(metrics) && isempty(conditions)
                metric_types = input_conversion(metrics');
                pose_metric_types = input_conversion(obj.poses{1}.metrics);
                
                for i = 1:size(obj.poses,1)
                    pose_data(i,:) = round(obj.poses{i}.pose',digit_tolerance);
                    filted_node_list(i,:) = obj.poses{i}.pose';
                end
                fixed_variables = round(fixed_variables,digit_tolerance);
                pose_data(:,[plot_axis]) = [];
                fixed_variables(:,[plot_axis]) = [];
                
                matched_poses_indices = find(ismember(pose_data,fixed_variables,'rows'));
                
                points_to_plot = filted_node_list(matched_poses_indices,:);
                for i = 1:size(matched_poses_indices,1)
                    for j = 1:num_metrics
                        metrics_indices = find(ismember(pose_metric_types,metric_types(j)));
                        point_color_matrix(j,i) = obj.poses{i}.metrics{metrics_indices,2};
                    end
                end
            else
                CASPR_log.Error('At least one of the metrics or conditions should be the input')
            end
            % universal plotting function for 2d/3d
            if ~isempty(matched_poses_indices)
                c_workspace = universal_plot(obj,plot_axis,points_to_plot,point_color_matrix);
                disp('Plot updated')
            else
                cla;
                disp('No results')
                c_workspace = [];
            end
        end
        
        % Plotting function to plot 2D/3D (subset of the) workspace with slider
        function f = plotWorkspaceSlider(obj,plot_axis,slide_axis,conditions, metrics, fixed_variables)
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
            for i = 1:num_metrics
                f(i) = figure(i);
                
                b(i) = uicontrol('Parent',f(i),'Style','slider','Position',[0,0,400,20],'value',fixed_variables(slide_axis), 'min',obj.grid.q_begin(slide_axis), 'max',obj.grid.q_end(slide_axis),...
                    'sliderstep',[1/(obj.grid.q_length(slide_axis)-1),1/(obj.grid.q_length(slide_axis)-1)]);
                current_fixed_variables = fixed_variables;
                
                b(i).Callback = @(es,ed) refreshdata(f(i),plotWorkspace(obj,plot_axis,conditions, metrics{i}, [current_fixed_variables(1:slide_axis-1),es.Value,current_fixed_variables(slide_axis+1:end)]));
            end
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
                    %% Hide the top axes
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
        
        function c_workspace = universal_plot(obj,plot_axis,points_to_plot,point_color_matrix)
            g = groot;
            if isempty(g.Children)
                for i = size(point_color_matrix,1):-1:1
                    c_workspace(i) = figure(i);
                end
            elseif ~isempty(g.Children)
                c_workspace = g.CurrentFigure;
            end
            if size(plot_axis,2) == 2 %plot 2D
                
                x = points_to_plot(:,plot_axis(1));
                y = points_to_plot(:,plot_axis(2));
                for i = 1:size(point_color_matrix,1)
                    c = point_color_matrix(i,:);
                    
                    if size(unique(c),2) ==1
                        c_workspace(i) = scatter(x,y,'filled','MarkerFaceColor','k');
                    else
                        c_workspace(i) = scatter(x,y,[],c','filled');
                        colorbar;
                    end
                    % plotting title and other stuff, nothing important
                    xlim(1.005*[obj.model.bodyModel.q_min(plot_axis(1)),obj.model.bodyModel.q_max(plot_axis(1))]);
                    ylim(1.005*[obj.model.bodyModel.q_min(plot_axis(2)),obj.model.bodyModel.q_max(plot_axis(2))]);
                    for j = 1:size(points_to_plot(1,:),2)
                        if j == plot_axis(1)
                            title_disp{j} = 'x-axis ';
                        elseif j == plot_axis(2)
                            title_disp{j} = 'y-axis ';
                        else
                            title_disp{j} = [num2str(points_to_plot(1,j)),' '];
                        end
                    end
                    %                     disp_message = ['Fig. ', num2str(i) ' shows the point(s) that fulfill the input condition(s) and metric(s) ', num2str(i)];
                    %                     disp(disp_message)
                    title_disp = cell2mat(title_disp);
                    title(title_disp)
                    clear title_disp;
                    %
                end
            elseif size(plot_axis,2) == 3 %Plot 3D
                x = points_to_plot(:,plot_axis(1));
                y = points_to_plot(:,plot_axis(2));
                z =  points_to_plot(:,plot_axis(3));
                
                for i = 1:size(point_color_matrix,1)
                    c = point_color_matrix(i,:);
                    if size(unique(c),2) ==1
                        figure(c_workspace(i));
                        c_workspace(i) = scatter3(x,y,z,'filled','MarkerFaceColor','k');
                    else
                        figure(c_workspace(i));
                        c_workspace(i) = scatter3(x,y,z,[],c','filled');
                        colorbar;
                    end
                    
                    % plotting title and other stuff, nothing important
                    xlim(1.005*[obj.model.bodyModel.q_min(plot_axis(1)),obj.model.bodyModel.q_max(plot_axis(1))]);
                    ylim(1.005*[obj.model.bodyModel.q_min(plot_axis(2)),obj.model.bodyModel.q_max(plot_axis(2))]);
                    zlim(1.005*[obj.model.bodyModel.q_min(plot_axis(3)),obj.model.bodyModel.q_max(plot_axis(3))]);
                    
                    for j = 1:size(points_to_plot(1,:),2)
                        if j == plot_axis(1)
                            title_disp{j} = 'x-axis ';
                        elseif j == plot_axis(2)
                            title_disp{j} = 'y-axis ';
                        elseif j == plot_axis(3)
                            title_disp{j} = 'z-axis ';
                        else
                            title_disp{j} = [num2str(points_to_plot(1,j)),' '];
                        end
                    end
                    %                     disp_message = ['Fig. ', num2str(i) ' shows the point(s) that fulfill the input condition(s) and metric ', num2str(i)];
                    %                     disp(disp_message)
                    title_disp = cell2mat(title_disp);
                    title(title_disp)
                    clear title_disp;
                    %
                end
            else
                CASPR_log.Error('Only 2D/3D plot is available. Try slider plot if more than 3 axis')
            end
        end
        
        % function to create the node_list variable
        function node_list = create_node_list(obj, conditions)
            pose_data = obj.poses;
            number_points = length(pose_data);
            % Create list of nodes
            node_list = zeros(number_points,1+obj.grid.n_dimensions);
            number_node = 0;
            n_conditions = size(conditions,2);
            if n_conditions == 1 && ~iscell(conditions)
                conditions = mat2cell(conditions,1);
            end
            for i = 1:n_conditions
                condition_types(i) = conditions{i}.type;
            end
            
            % For each point
            for i = 1:number_points
                % Find out the poses that fulfill the condition(s)
                if(~isempty(pose_data{i}))
                    
                    n_constraints = size(pose_data{i}.conditions,1); 
                    if n_constraints ~=0
                    for j = 1:n_constraints
                        pose_condition_types(j) = pose_data{i}.conditions{j,1}.type;
                    end
                    if(all(ismember(condition_types,pose_condition_types)))
                        number_node = number_node + 1;
                        node_list(number_node,:) = [number_node,pose_data{i}.pose'];
                    end
                    end
                end
            end
            % Resize to the correct size
            node_list = node_list(1:number_node,:);
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

% function to handle numerous inputs to processable inputs
function output_types = input_conversion(input)

num_input= size(input,1);

if num_input == 1 && ~iscell(input)
    input = mat2cell(input,1);
end
for i = 1:num_input
    output_types(i) = input{i,1}.type;
end
end