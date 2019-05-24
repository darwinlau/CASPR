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
        
        function createWorkspacePointGraph(obj,conditions,metrics)
            % Create list of nodes and graph in point representation
            obj.point_node(:).node_list = create_point_node_list(obj, conditions);
            obj.point_node(:).graph_rep =  create_point_graph_rep(obj, obj.point_node.node_list, metrics);
            
        end
        
        function createWorkspaceRayGraph(obj,conditions,metrics)
            
            % Create list of nodes and graph in ray representation
            obj.ray_node(:).node_list = create_ray_node_list(obj, conditions);
            obj.ray_node(:).graph_rep =  create_ray_graph_rep(obj, obj.ray_node.node_list, metrics);
            
            
        end
        
        function plotPointGraph(obj,metrics)
            for i = 1:size(metrics)
                
            end
        end
        
        function plotPointWorkspace()
        end
        
        function plotRayGraph()
            
        end
        
        function plotRayWorkspace(obj,node_list,plot_axis,fixed_variables)
            plot_data_index = find(ismember(cell2mat(node_list(:,end)),plot_axis));
            plot_data = node_list(plot_data_index,[plot_axis+1,end]);
            
            fixed_variable_column_index = 1:size(node_list,2)-2;
            fixed_variable_column_index([plot_axis]) = [];
            fixed_variable_data = node_list(plot_data_index,[fixed_variable_column_index+1]);
            
            fixed_variables([plot_axis]) = [];
            
            variables_matched_index = find(ismember(cell2mat(fixed_variable_data),fixed_variables,'rows'));
            
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
                hold on
            end
            
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
        function graph_rep = create_point_graph_rep(obj, node_list, metrics)
            
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
                        node_list{number_node,1} = i; % node number
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
        function graph_rep = create_ray_graph_rep(obj, node_list,metrics)
            graph_rep = [];metric_value = [];
            
            for j = 1:size(node_list,1)
                current_variable = find(ismember([node_list{:,end}],node_list{j,end}));
                starting_index = current_variable(end)+1;
                for i = starting_index:size(node_list,1)
                    % check parallel ray
                    if node_list{i,end} ~= node_list{j,end}
                        [intersected_point,min_dist] = check_intersection({node_list{j,2:end}},{node_list{i,2:end}});
                        if ~isempty(intersected_point)
                            % evaluate intersected points
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

