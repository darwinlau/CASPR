% A container class to hold workspace result for a point pose
%
% Author        : Paul Cheng
% Created       : 2019
% Description    : This class contains the known information obtained
% through workspace analysis for all poses
classdef RayWorkspace < handle
    properties (SetAccess = protected)
        model
        grid                        % Grid object for brute force workspace
    end
    
    properties
        rays                        % Cell array
        point_node = []             % use intersected point as node
        ray_node = []               % use intersected point as node
    end
    
    methods
        function pw = RayWorkspace(model, grid)
            pw.model = model;
            pw.grid = grid;
            pw.rays = cell(grid.n_points,1);
            pw.ray_node = struct('graph_rep', [], 'node_list', []);
            pw.point_node = struct('graph_rep', [], 'node_list', []);
            % graph_rep        % The graph representation for the workspace
            % sturcture of grpah_rep:[node_num,node_num,min_distance,metric_1,...metric_n]
            %node_list        % A list of all nodes
        end
        
        function createWorkspacePointGraph(obj,conditions)
            % Create list of nodes and graph in point representation
            obj.point_node(:).node_list = create_point_node_list(obj, conditions);
            obj.point_node(:).graph_rep =  create_point_graph_rep(obj, obj.point_node.node_list);
            
        end
        
        function intersected_point_set = createWorkspaceRayGraph(obj,conditions,metrics)
            
            % Create list of nodes and graph in ray representation
            obj.ray_node(:).node_list = create_ray_node_list(obj, conditions);
            [obj.ray_node(:).graph_rep,intersected_point_set] =  create_ray_graph_rep(obj, obj.ray_node.node_list, metrics);
            
            
        end
        
        function point_graph = plotPointGraph(obj,conditions,metrics)
            createWorkspacePointGraph(obj,conditions);
            if ~isempty(metrics)
            for i = 1:size(obj.point_node.node_list,1)
                wp(i,:) = PointWorkspaceElement(obj.point_node.node_list(i,2:end), obj.model, [], metrics);
            end
            for i = 1:size(metrics,2)
                for j = 1:size(obj.point_node.node_list,1)
                    color_matrix(:,j) = wp(j).metrics{i,2};
                end
                G = graph(obj.point_node.graph_rep(:,1),obj.point_node.graph_rep(:,2));
                edge_color_matrix = obj.point_node.graph_rep(:,3);
                for k = 1:G.numnodes
                    node_color_matrix(k) = color_matrix(k);
                end
                title_str = [];
                plot2ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
                
            end
            else
                G = graph(obj.point_node.graph_rep(:,1),obj.point_node.graph_rep(:,2));
                edge_color_matrix = obj.point_node.graph_rep(:,3);
                for k = 1:G.numnodes
                    node_color(k,:) = [0 0 0];
                end
                title_str = [];
                plot1ColorGraph(G,node_color,edge_color_matrix,title_str);                
            end
        end
        
        function plotPointWorkspace(obj,plot_axis,conditions,fixed_variables)
            rounding_digit = 4; % remove numerical error from input, change it if not accurate enough
            fixed_variables([plot_axis]) = [];
            fixed_variables = round(fixed_variables,rounding_digit);
            if size(fixed_variables,2) + size(plot_axis,2) ~= obj.model.numDofs
                CASPR_log.Error('Not enought number of fixed axis')
            end
            fixed_axis = 1:obj.model.numDofs;
            fixed_axis(plot_axis) = []; 
            node_list = round(create_point_node_list(obj, conditions),rounding_digit);
            if ~isempty(fixed_variables)
            plot_data_index = find(ismember(node_list(:,1 + fixed_axis),fixed_variables,'rows'));
            elseif size(plot_axis,2) == obj.model.numDofs
                plot_data_index = node_list(:,1); 
            end
            plot_data = node_list(plot_data_index,1+[plot_axis]);
            
             if isempty(plot_data)
                CASPR_log.Error('No available plot, try to change fixed variable value')
            end
            figure;
            if size(plot_axis,2) == 3 %3D plot                
                    scatter3(plot_data(:,1),plot_data(:,2),plot_data(:,3),'filled', 'MarkerEdgeColor',[0 0 0],...
              'MarkerFaceColor',[0 0 0]);
                    xlabel(['Variable: ',num2str(plot_axis(1))]);
                    ylabel(['Variable: ',num2str(plot_axis(2))]);
                    zlabel(['Variable: ',num2str(plot_axis(3))]);                    
            elseif size(plot_axis,2) == 2 %2D plot
                
                    scatter(plot_data(:,1),plot_data(:,2),'filled', 'MarkerEdgeColor',[0 0 0],...
              'MarkerFaceColor',[0 0 0]);
                    xlabel(['Variable: ',num2str(plot_axis(1))]);
                    ylabel(['Variable: ',num2str(plot_axis(2))]);
                    hold on
               
            else
                CASPR_log.Error('Only 3D/2D plot is available.')
            end
            
            title(['Fixed variables: ',num2str(fixed_variables)]);
        
    end
        
        function plotRayGraph(obj,conditions,metrics)
            intersected_point_set = createWorkspaceRayGraph(obj,conditions,metrics);
            
            G = graph(obj.ray_node.graph_rep(:,1),obj.ray_node.graph_rep(:,2));
            %graph for n variable value for intersection point(edge) and ray(node)
            for i = 1:obj.model.numDofs
                for j = 1:size(obj.ray_node.node_list,1) 
                if i == obj.ray_node.node_list{j,end}
                    node_color_matrix(j) =  0;
                else
                    node_color_matrix(j) = obj.ray_node.node_list{j,i+1};
                end
                end
                for j = 1:size(G.Edges,1)
                    edge_color_matrix(j) = intersected_point_set(j,i);
                end
                title_str = ['Variable ' num2str(i)];
                plot2ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
            end            
            
            %graph for length of ray(node) and min distance to the ray(ray)
            for i = 1:size(obj.ray_node.node_list,1)
                node_color_matrix(i) =  norm(obj.ray_node.node_list{i,obj.ray_node.node_list{i,end}+1});
            end
            edge_color_matrix = obj.ray_node.graph_rep(:,3);
            title_str = ['Ray length and boundary distance'];
            plot2ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
            node_color_matrix = [];edge_color_matrix = [];
            
            %graph for number of intersection            
            for i = 1:size(obj.ray_node.node_list,1)
                node_color_matrix(i) = sum(sum(ismember(obj.ray_node.graph_rep(:,[1 2]),i)));
            end
            for j = 1:size(G.Edges,1)
                edge_color_matrix(j,:) = [0.5 0.5 0.5];
            end
            title_str = 'Number of intersection'
            plot1ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
            node_color_matrix = [];edge_color_matrix = [];
            
            %graph for metrics
            for i = 1:size(metrics,2)
                for j = 1:size(obj.ray_node.node_list,1)
                    node_color_matrix(j,:) = [0.5 0.5 0.5];
                end
                edge_color_matrix = obj.ray_node.graph_rep(:,i+3);
                
                title_str = ['Metric ', num2str(i)];
                plot1ColorGraph(G,node_color_matrix,edge_color_matrix,title_str)
            end
            node_color_matrix = [];edge_color_matrix = [];
        end
        
        function plotRayWorkspace(obj,plot_axis,conditions,fixed_variables)
            rounding_digit = 4; % remove numerical error from input, change it if not accurate enough
            fixed_variables([plot_axis]) = [];
            fixed_variables = round(fixed_variables,rounding_digit);
            if size(fixed_variables,1) + size(plot_axis,2) ~= obj.model.numDofs
                CASPR_log.Error('Not enought number of fixed axis')
            end
            node_list = create_ray_node_list(obj, conditions);
            plot_data_index = find(ismember(cell2mat(node_list(:,end)),plot_axis));
            plot_data = node_list(plot_data_index,[plot_axis+1,end]);
            
            fixed_variable_column_index = 1:size(node_list,2)-2;
            fixed_variable_column_index([plot_axis]) = [];
            fixed_variable_data = node_list(plot_data_index,[fixed_variable_column_index+1]);
            
             if ~isempty(fixed_variables)
                 fixed_variable_data = round(fixed_variable_data,rounding_digit);
            variables_matched_index = find(ismember(cell2mat(fixed_variable_data),fixed_variables,'rows'));
            elseif size(plot_axis,2) == obj.model.numDofs
                variables_matched_index = [obj.ray_node.node_list{:,1}]'; 
            end
            if isempty(variables_matched_index)
                CASPR_log.Error('No available plot, try to change fixed variable value')
            end
            figure;
            if size(plot_axis,2) == 3 %3D plot
                for i = 1:size(variables_matched_index,1)
                    if plot_data{variables_matched_index(i),end} == plot_axis(1)
                        x = plot_data{variables_matched_index(i),1};
                        y = ones(1,2)*plot_data{variables_matched_index(i),2};
                        z = ones(1,2)*plot_data{variables_matched_index(i),3};
                    elseif plot_data{variables_matched_index(i),end} == plot_axis(2)
                        x = ones(1,2)*plot_data{variables_matched_index(i),1};
                        y = plot_data{variables_matched_index(i),2};
                        z = ones(1,2)*plot_data{variables_matched_index(i),3};
                    elseif plot_data{variables_matched_index(i),end} == plot_axis(3)
                        x = ones(1,2)*plot_data{variables_matched_index(i),1};
                        y = ones(1,2)*plot_data{variables_matched_index(i),2};
                        z = plot_data{variables_matched_index(i),3};
                    end
                    plot3(x,y,z,'k');
                     xlabel(['Variable: ',num2str(plot_axis(1))]);
                    ylabel(['Variable: ',num2str(plot_axis(2))]);
                    zlabel(['Variable: ',num2str(plot_axis(3))]);
                    hold on
                end
            elseif size(plot_axis,2) == 2 %2D plot
                 for i = 1:size(variables_matched_index,1)
                    if plot_data{variables_matched_index(i),end} == plot_axis(1)
                        x = plot_data{variables_matched_index(i),1};
                        y = ones(1,2)*plot_data{variables_matched_index(i),2};
                    elseif plot_data{variables_matched_index(i),end} == plot_axis(2)
                        x = ones(1,2)*plot_data{variables_matched_index(i),1};
                        y = plot_data{variables_matched_index(i),2};
                    end
                    plot(x,y,'k');
                    xlabel(['Variable: ',num2str(plot_axis(1))]);
                    ylabel(['Variable: ',num2str(plot_axis(2))]);
                    hold on
                end
            else
                CASPR_log.Error('Only 3D/2D plot is available.')                
            end
            title(['Fixed variables: ',num2str(fixed_variables)]);
        end
        
    end
    
    methods(Access = private)
        
        % function to create the node_list variable for point representation
        function node_list = create_point_node_list(obj, conditions)
            
            intersected_ray = {};ref_intersected_point = [];
            rays_seg = create_ray_node_list(obj, conditions);%get every rays as node
            for i = 1:size(rays_seg,1)
                current_variable = find(ismember([rays_seg{:,end}],rays_seg{i,end}));
                starting_index = current_variable(end)+1;
                ray_A = rays_seg(i,2:end);
                for j = starting_index:size(rays_seg,1)
                    ray_B = rays_seg(j,2:end);
                    [intersected_point,~] = check_intersection(ray_A,ray_B);
                    intersected_point = round(intersected_point,10);
                    if ~isempty(intersected_point)
                        intersected_ray(end+1,:) = {rays_seg{i,1},rays_seg{j,1}};
                        ref_intersected_point(end+1,:) = intersected_point;
                    end
                end
            end
            unique_points = unique(ref_intersected_point,'rows');
            node_list = cell(size(unique_points,1),size(unique_points,2)+obj.model.numDofs);%[node_number,intersected_point,ray_number]
            for i = 1:size(unique_points,1)
                repeated_point_index = find(ismember(ref_intersected_point,unique_points(i,:),'rows'));
                int_ray_index = [];
                %                 int_ray_index = unique(cell2mat(intersected_ray([repeated_point_index],:)));
                %                 if size(int_ray_index,1)>1
                %                     int_ray_index = int_ray_index';
                %                 end
                node = num2cell([i,unique_points(i,:),int_ray_index]);
                node_list(i,1:size(node,2)) = node;
                clear node
            end
            node_list = cell2mat(node_list);
        end
        
        % function to create the graph_rep variable for point representation
        function graph_rep = create_point_graph_rep(obj, node_list)
            
            graph_rep = [];
            for k = 1:size(node_list,1)
                for i = 1:size(obj.grid.dim_disc_ia,1)
                    dimension_index = obj.grid.dim_disc_ia(i);
                    delta_q_vector = zeros(1,obj.model.numDofs);
                    delta_q = obj.grid.delta_q(dimension_index);
                    delta_q_vector(obj.grid.dim_disc_ia(i)) = delta_q;
                    for j = 1:2
                        if j == 1
                            nearby_point = node_list(k,2:obj.model.numDofs+1) + delta_q_vector;
                        else
                            nearby_point = node_list(k,2:obj.model.numDofs+1) - delta_q_vector;
                        end
                        neighbour_point_index = find(ismember(node_list(:,2:obj.model.numDofs+1),nearby_point,'rows'));
                        
                        if ~isempty(neighbour_point_index)
                            if ~ismember(neighbour_point_index,node_list(1:k,1))
                                %                                 graph_rep(end+1,:) =
                                %                                 [node_list{k,1},neighbour_point_index,delta_q_vector];
                                %                                 % for grpah search, this one may be more
                                %                                 usefull since it sepearte the distance in
                                %                                 different dimension
                                graph_rep(end+1,:) = [node_list(k,1),neighbour_point_index,delta_q];
                            end
                        end
                    end
                end
            end
        end
        
        % function to create the node_list variable for ray representation
        function node_list = create_ray_node_list(obj, conditions)
            % covert the input conditions to same format as the ray
            % condition
            conditions_types = input_conversion(conditions);
            number_node = 0;
            node_list = cell(1,obj.model.numDofs+2);
            for i = 1:size(obj.rays,1)
                % Determine the ray that has same condtions as input
                if(~isempty(obj.rays{i}))
                    num_conditions = size(obj.rays{i}.conditions,1);
                    intervals = [];
                    
                    for j = 1:num_conditions
                        ray_condition_types(j) = obj.rays{i}.conditions{j,1};
                    end
                    
                    if(all(ismember(conditions_types,ray_condition_types)))
                        intervals = obj.rays{i}.conditions{j,2};
                    end
                    for j = 1:size(intervals,1)
                        number_node = number_node + 1;
                        node_list{number_node,1} = number_node; % node number
                        kk = 1;
                        for k = 2:size(node_list,2)-1
                            if k ~= obj.rays{i}.free_variable_index+1
                                node_list{number_node,k} =  obj.rays{i}.fixed_variables(kk);
                                kk = kk + 1;
                            else
                                node_list{number_node,k} =  intervals(j,:);
                            end
                        end
                        node_list{number_node,end} = obj.rays{i}.free_variable_index;
                    end
                end
            end
        end
        
        % function to create the graph_rep variable for ray representation
        function [graph_rep,intersected_point_set] = create_ray_graph_rep(obj, node_list,metrics)
            graph_rep = [];metric_value = [];
            intersected_point_set = [];
            for j = 1:size(node_list,1)
                current_variable = find(ismember([node_list{:,end}],node_list{j,end}));
                starting_index = current_variable(end)+1;
                for i = starting_index:size(node_list,1)
                    % check parallel ray
                    if node_list{i,end} ~= node_list{j,end}
                        
                        [intersected_point,min_dist] = check_intersection({node_list{j,2:end}},{node_list{i,2:end}});
                        if ~isempty(intersected_point)
                            % evaluate intersected points
                            intersected_point_set = [intersected_point_set;intersected_point];
                            if ~isempty(metrics)
                                for k = 1:size(metrics,2)
                                    point_metric = PointWorkspaceElement(intersected_point, obj.model, [], metrics(k));
                                    metric_value(k) = point_metric.metrics{2};
                                end
                            end
                            graph_rep = [graph_rep;j,node_list{i,1},min_dist,metric_value];
                        end
                    end
                end
            end
        end
        
    end
    
end
% function to handle numerous inputs to processable inputs
function output_types = input_conversion(input)
num_input= size(input,2);

if num_input == 1 && ~iscell(input)
    input = mat2cell(input,1);
end
for i = 1:num_input
    output_types(i) = input{i}.type;
end
end

% function to check intersection  and distance between two co-plannar rays
function [intersected_point,min_dist] = check_intersection(ray_1,ray_2)
% check if co-planar

for i = 1:size(ray_2,2)-1
    if i == ray_2{end} || i == ray_1{end}
        intersected_point(i) = Inf;
    else
        pt_B = ray_2{i};
        pt_A = ray_1{i};
        if pt_A ~= pt_B
            co_planar_flag = 0;
            intersected_point = [];
            min_dist = [];
            break
        else
            co_planar_flag = 1;
            intersected_point(i) = pt_A;
        end
    end
end
% co_planar_flag
if (co_planar_flag)
    seg_A_x = ray_1{ray_1{end}};
    seg_A_y = ones(1,2)*ray_1{ray_2{end}};
    
    seg_B_x = ones(1,2)*ray_2{ray_1{end}};
    seg_B_y = ray_2{ray_2{end}};
    %     clf
    %     plot(seg_A_x,seg_A_y);hold on
    %     plot(seg_B_x,seg_B_y);
    
    [x_intersected,y_intersected] = polyxpoly(seg_A_x,seg_A_y,seg_B_x,seg_B_y);
    int_coor = [x_intersected,y_intersected];
    
    
    if ~isempty(x_intersected) && ~isempty(y_intersected)
        min_dist = min([abs(seg_A_x(1)-x_intersected),abs(seg_A_x(2)-x_intersected),...
            abs(seg_B_y(1)-y_intersected),abs(seg_B_y(2)-y_intersected)]);
        
        if ray_1{end} > ray_2{end}
            int_coor = fliplr(int_coor);
        end
        j = 1;
        for i = 1:size(intersected_point,2)
            if intersected_point(i) == Inf
                intersected_point(i) = int_coor(j);
                j = j + 1;
            end
        end
    else
        min_dist = [];intersected_point = [];
    end
end
end

% function to plot two color bar graph
function graph = plot2ColorGraph(G,node_color_matrix,edge_color_matrix,title_str)
figure
for i = 1:2
    ax(i) = axes;
    if i == 1
        graph = plot(G,'MarkerSize',2);
        graph.NodeCData = node_color_matrix';
        graph.EdgeColor = 'none';
    else
        graph = plot(G,'MarkerSize',2);
        graph.NodeColor = 'none';
        graph.EdgeCData = edge_color_matrix';
    end
end
linkaxes([ax(1),ax(2)]);
ax(2).Visible = 'off';
ax(2).XTick = [];
ax(2).YTick = [];
colormap(ax(1),cool)
colormap(ax(2),autumn);
set([ax(1),ax(2)],'Position',[.17 .11 .685 .815]);
cb1 = colorbar(ax(1),'Position',[.14 .11 .03 .815]);
cb2 = colorbar(ax(2),'Position',[.84 .11 .03 .815]);
dim = [.42 .7 .3 .3];
annotation('textbox',dim,'EdgeColor','none','String',title_str,'FitBoxToText','on');

end

% function to plot one color bar graph
function graph = plot1ColorGraph(G,node_color,edge_color,title_str)
figure
graph = plot(G,'MarkerSize',2);
if size(node_color,1) == 1
    graph.NodeCData = node_color';
else
    graph.NodeColor = node_color;
end
if size(edge_color,2) == 1
    graph.EdgeCData = edge_color';
else    
    graph.EdgeColor = edge_color;
end
colormap(autumn);
colorbar;
title(title_str);
end
