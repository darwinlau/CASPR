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
    
    properties(Hidden = true)
        tolerance = 5;
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
        
        function createWorkspacePointGraph(obj)
            % Create list of nodes and graph in point representation
            obj.point_node.node_list = obj.create_point_node_list();
            obj.point_node.graph_rep = obj.create_point_graph_rep();
            
        end
        
        function intersected_point_set = createWorkspaceRayGraph(obj, metrics)
            % Create list of nodes and graph in ray representation
            if (isempty(obj.ray_node.node_list))
                obj.create_ray_node_list();
            end
            [obj.ray_node.graph_rep,intersected_point_set] =  create_ray_graph_rep(obj, obj.ray_node.node_list, metrics);
            
            
        end
        
        function point_graph = plotPointGraph(obj, metrics)
            
            obj.createWorkspacePointGraph();
            
            if ~isempty(metrics)
                for i = 1:size(obj.point_node.node_list,1)
                    wp(i,:) = PointWorkspaceElement(obj.point_node.node_list(i,2:end), obj.model, [], metrics);
                end
                for i = 1:size(metrics,2)
                    for j = 1:size(obj.point_node.node_list,1)
                        color_matrix(:,j) = wp(j).metrics{i,2};
                    end
                    G = graph(obj.point_node.graph_rep(:,1),obj.point_node.graph_rep(:,2));
                    empty_node_index = find(degree(G)==0);
                    G = rmnode(G,empty_node_index);
                    edge_color_matrix = obj.point_node.graph_rep(:,3);
                    for k = 1:G.numnodes
                        node_color_matrix(k) = color_matrix(k);
                    end
                    title_str = [];
                    obj.plot2ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
                    
                end
            else
                G = graph(obj.point_node.graph_rep(:,1),obj.point_node.graph_rep(:,2));
                edge_color_matrix = obj.point_node.graph_rep(:,3);
                for k = 1:G.numnodes
                    node_color(k,:) = [0 0 0];
                end
                title_str = [];
                obj.plot1ColorGraph(G,node_color,edge_color_matrix,title_str);
            end
        end
        
        function result_fig = plotPointWorkspace(obj,plot_axis,fixed_variables)
            rounding_digit = 4; % remove numerical error from input, change it if not accurate enough
            fixed_variables([plot_axis]) = [];
            fixed_variables = fix(fixed_variables*10^rounding_digit)/10^rounding_digit;
            if size(fixed_variables,2) + size(plot_axis,2) ~= obj.model.numDofs
                CASPR_log.Error('Not enought number of fixed axis')
            end
            fixed_axis = 1:obj.model.numDofs;
            fixed_axis(plot_axis) = [];
            node_list = obj.create_point_node_list();
            %             node_list = obj.point_node.node_list; % use this if there are
            %             node_list already
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
                result_fig = scatter3(plot_data(:,1),plot_data(:,2),plot_data(:,3),'filled', 'MarkerEdgeColor',[0 0 0],...
                    'MarkerFaceColor',[0 0 0]);
                xlabel(['Variable: ',num2str(plot_axis(1))]);
                ylabel(['Variable: ',num2str(plot_axis(2))]);
                zlabel(['Variable: ',num2str(plot_axis(3))]);
            elseif size(plot_axis,2) == 2 %2D plot
                
                result_fig = scatter(plot_data(:,1),plot_data(:,2),'filled', 'MarkerEdgeColor',[0 0 0],...
                    'MarkerFaceColor',[0 0 0]);
                xlabel(['Variable: ',num2str(plot_axis(1))]);
                ylabel(['Variable: ',num2str(plot_axis(2))]);
                hold on
                
            else
                CASPR_log.Error('Only 3D/2D plot is available.')
            end
            
            title(['Fixed variables: ',num2str(fixed_variables)]);
            
        end
        
        function plotRayGraph(obj, metrics)
            intersected_point_set = createWorkspaceRayGraph(obj, metrics);
            
            G = graph(obj.ray_node.graph_rep(:,1),obj.ray_node.graph_rep(:,2));
            %remove zero intersection node
            empty_node_index = find(degree(G)==0);
            G = rmnode(G,empty_node_index);
            %graph for n variable value for intersection point(edge) and ray(node)
            replete_node_index = find(degree(G)~=0);
            for i = 1:obj.model.numDofs
                for j = 1:size(replete_node_index,1)
                    if i == obj.ray_node.node_list{j,2}
                        node_color_matrix(j) =  0;
                    else
                        node_color_matrix(j) = obj.ray_node.node_list{replete_node_index(j),i+2};
                    end
                end
                for j = 1:size(G.Edges,1)
                    edge_color_matrix(j) = intersected_point_set(j,i);
                end
                title_str = ['Variable ' num2str(i)];
                obj.plot2ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
            end
            
            %graph for length of ray(node) and min distance to the ray(ray)
            for i = 1:size(replete_node_index,1)
                node_color_matrix(i) =  norm(obj.ray_node.node_list{replete_node_index(i),obj.ray_node.node_list{replete_node_index(i),2}+1});
            end
            edge_color_matrix = obj.ray_node.graph_rep(:,3);
            title_str = ['Ray length and boundary distance'];
            obj.plot2ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
            node_color_matrix = [];edge_color_matrix = [];
            
            %graph for number of intersection
            for i = 1:size(replete_node_index,1)
                node_color_matrix(i) = sum(sum(ismember(obj.ray_node.graph_rep(:,[1 2]),replete_node_index(i))));
            end
            for j = 1:size(G.Edges,1)
                edge_color_matrix(j,:) = [0.5 0.5 0.5];
            end
            title_str = 'Number of intersection';
            obj.plot1ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
            node_color_matrix = [];edge_color_matrix = [];
            
            %graph for metrics
            for i = 1:size(metrics,2)
                for j = 1:size(replete_node_index,1)
                    node_color_matrix(j,:) = [0.5 0.5 0.5];
                end
                edge_color_matrix = obj.ray_node.graph_rep(:,replete_node_index(i)+3);
                
                title_str = ['Metric ', num2str(i)];
                obj.plot1ColorGraph(G,node_color_matrix,edge_color_matrix,title_str);
            end
            node_color_matrix = [];edge_color_matrix = [];
        end
        
        % Function to plot the workspace in 2D or 3D. For robots with more
        % than 3 DoFs, values of the fixed DoFs need to be provided to
        % lower its dimension to 2D or 3D.
        %
        % Inputs:
        %   - dofs_to_plot: The array of the joint pose (q) indices to plot
        %       in 2-D or 3-D (in the order of XYZ)
        %   - fixed_var_val: The array of the values for the fixed
        %       variables when the DoF of the robot is greater than the
        %       dimension of the plot. Note that the dimension of the array
        %       must the same as the DoF of the robot, the fixed values for
        %       the DoFs to be plotted will be ignored.
        function w_handles = plotRayWorkspace(obj, dofs_to_plot, fixed_var_val, is_plot_current)
            digit_tolerance = 4; % remove numerical error from input, change it if not accurate enough
            
            % Start with a set of checking conditions
            CASPR_log.Assert(length(dofs_to_plot) == 2 || length(dofs_to_plot) == 3, 'Number of DoFs to plot must be 2 or 3, otherwise it cannot be plotted.');
            CASPR_log.Assert(length(dofs_to_plot) <= obj.model.numDofs, 'The number of DoFs to plot is more than the degrees of freedom available.');
            if (length(dofs_to_plot) > obj.model.numDofs)
                CASPR_log.Assert(nargin > 4, 'Must input ''fixed_var_val'' if the DoFs to plot does not cover all DoFs.');
                CASPR_log.Assert(length(fixed_var_val) == obj.model.numDofs, 'Input ''fixed_var_val'' must have the same dimension as the DoFs of the robot.');
            end
            
            if nargin < 3
                fixed_var_val = zeros(1, obj.model.numDofs);
            end
            if nargin < 4
                is_plot_current = false;
            end
            
            fixed_var_val = reshape(fixed_var_val, [1,obj.model.numDofs]);
            fixed_var_val = round(fixed_var_val, digit_tolerance);
            fixed_var_val(:, dofs_to_plot) = [];
            
            if (isempty(obj.ray_node.node_list))
                node_list = obj.create_ray_node_list();
            else
                node_list = obj.ray_node.node_list;
            end
            
            plot_data_index = find(ismember(cell2mat(node_list(:,2)), dofs_to_plot));
            plot_data = node_list(plot_data_index, [2,2+dofs_to_plot]);
            
            fixed_variable_column_index = 3:size(node_list,2);
            fixed_variable_column_index(dofs_to_plot) = [];
            fixed_variable_data = node_list(plot_data_index, [fixed_variable_column_index]);
            if ~isempty(fixed_var_val)
                fixed_variable_data = cellfun(@(x)round(x, digit_tolerance), fixed_variable_data);
                variables_matched_index = find(ismember(fixed_variable_data, fixed_var_val, 'rows'));
            elseif size(dofs_to_plot, 2) == obj.model.numDofs
                variables_matched_index = [obj.ray_node.node_list{:,1}]';
            end
            if isempty(variables_matched_index)
                CASPR_log.Warn('No available plot, try to change fixed variable value')
            end
            figure;
            if size(dofs_to_plot, 2) == 3 %3D plot
                for i = 1:size(variables_matched_index,1)
                    if plot_data{variables_matched_index(i),1} == dofs_to_plot(1)
                        x = plot_data{variables_matched_index(i),2}';
                        y = ones(1,2)*plot_data{variables_matched_index(i),3}';
                        z = ones(1,2)*plot_data{variables_matched_index(i),4}';
                    elseif plot_data{variables_matched_index(i),1} == dofs_to_plot(2)
                        x = ones(1,2)*plot_data{variables_matched_index(i),2}';
                        y = plot_data{variables_matched_index(i),3}';
                        z = ones(1,2)*plot_data{variables_matched_index(i),4}';
                    elseif plot_data{variables_matched_index(i),1} == dofs_to_plot(3)
                        x = ones(1,2)*plot_data{variables_matched_index(i),2}';
                        y = ones(1,2)*plot_data{variables_matched_index(i),3}';
                        z = plot_data{variables_matched_index(i),4}';
                    end
                    w_handles(i) = plot3(x,y,z,'k');
                    % plotting title and other stuff, nothing important
                    xlim(1.005*[obj.model.bodyModel.q_min(dofs_to_plot(1)),obj.model.bodyModel.q_max(dofs_to_plot(1))]);
                    ylim(1.005*[obj.model.bodyModel.q_min(dofs_to_plot(2)),obj.model.bodyModel.q_max(dofs_to_plot(2))]);
                    zlim(1.005*[obj.model.bodyModel.q_min(dofs_to_plot(3)),obj.model.bodyModel.q_max(dofs_to_plot(3))]);
                    xlabel(sprintf('q_%d', dofs_to_plot(1)));
                    ylabel(sprintf('q_%d', dofs_to_plot(2)));
                    zlabel(sprintf('q_%d', dofs_to_plot(3)));
                    hold on;
                end
                hold off;
            elseif size(dofs_to_plot, 2) == 2 %2D plot
                for i = 1:size(variables_matched_index,1)
                    if plot_data{variables_matched_index(i),1} == dofs_to_plot(1)
                        x = plot_data{variables_matched_index(i),2}';
                        y = ones(1,2)*plot_data{variables_matched_index(i),3}';
                    elseif plot_data{variables_matched_index(i),1} == dofs_to_plot(2)
                        x = ones(1,2)*plot_data{variables_matched_index(i),2}';
                        y = plot_data{variables_matched_index(i),3}';
                    end
                   w_handles(i) =  plot(x,y,'k');
                    xlim(1.005*[obj.model.bodyModel.q_min(dofs_to_plot(1)),obj.model.bodyModel.q_max(dofs_to_plot(1))]);
                    ylim(1.005*[obj.model.bodyModel.q_min(dofs_to_plot(2)),obj.model.bodyModel.q_max(dofs_to_plot(2))]);
                    xlabel(sprintf('q_%d', dofs_to_plot(1)));
                    ylabel(sprintf('q_%d', dofs_to_plot(2)));
                    hold on
                end
                hold off;
            else
                CASPR_log.Error('Only 3D/2D plot is available.')
            end
            title(['Fixed variables: ',num2str(fixed_var_val)]);
        end
        
    end
    
    methods(Access = private)
        % function to create the node_list variable for point representation
        function node_list = create_point_node_list(obj)
            
            if (isempty(obj.ray_node.node_list))
                rays_seg = obj.create_ray_node_list();%get every rays as node
            else
                rays_seg = obj.ray_node.node_list;
            end
            ternimal_index = find(ismember([rays_seg{:,2}],rays_seg{end,2}));
            ternimal_index = ternimal_index(1);
            point_node_list = []; connectivity = [];
            point_node_num = 0; points_on_ray = [];
            for cur_ind = 1:size(rays_seg,1) - ternimal_index
                intersected_pt_order = zeros(1,obj.grid.n_dimensions);
                ray_A_node_num = rays_seg{cur_ind,1};
                ray_A = rays_seg(cur_ind,3:end);
                ray_A_free_val = rays_seg{cur_ind,2};
                intersected_pt_order(ray_A_free_val) = 1;
                if  obj.model.bodyModel.numDofs >2
                    for j = rays_seg{cur_ind,2}+1:obj.grid.n_dimensions
                        
                        ray_B_free_val = j;
                        intersected_pt_order(ray_B_free_val) = 1;
                        compare_ray_set = rays_seg([rays_seg{:,2}] == j,3:end);
                        if ~isempty(compare_ray_set)
                            ray_B_set_node_num = rays_seg([rays_seg{:,2}] == j,1);
                            [co_plannar_node_num,co_plannar_ind,plane] = obj.co_plannar_search(ray_A,compare_ray_set,rays_seg{cur_ind,2},j,ray_B_set_node_num);
                            [intersected_points,min_dist,intersected_node_num]  = obj.find_intersection(ray_A{ray_B_free_val},ray_A{ray_A_free_val},compare_ray_set(co_plannar_ind,ray_A_free_val),compare_ray_set(co_plannar_ind,j),...
                                ray_B_free_val,ray_A_free_val,plane,obj.grid.n_dimensions,co_plannar_node_num);
                            if ~isempty(intersected_points)
                                point_node_list = [point_node_list;intersected_points];
                            end
                        end
                    end
                end
            end
            [~,ia,~] = unique(round(point_node_list,obj.tolerance),'rows');
            
            node_list = [[1:size(ia,1)]', point_node_list(ia,:)];
           
        end
        %% function to find co-planar ray
        function [co_plannar_node_num,co_plannar_ind,co_plannar]= co_plannar_search(~,ray_A,ray_B_set,free_val_A,free_val_B,node_number)
            ray_A(:,free_val_B) = []; ray_A(:,free_val_A) = []; ray_A = cell2mat(ray_A);
            ray_B_set(:,free_val_B) = []; ray_B_set(:,free_val_A) = [];ray_B_set = cell2mat(ray_B_set);
            co_plannar_node_num = cell2mat(node_number(round(vecnorm([ray_B_set - ray_A]'),5) == 0));
            co_plannar_ind = find(round(vecnorm([ray_B_set - ray_A]'),5) == 0)';
            co_plannar = ray_A;
        end
        
        %% function to check intersection
        function [intersected_points,min_dist,intersected_node_num] = find_intersection(~,ray_A_val,range_A,ray_B_val,range_B,ray_A_val_ind,ray_B_val_ind,plane,pt_size,co_plannar_node_num)
            range_B = cell2mat(range_B);
            ray_B_val = cell2mat(ray_B_val); 
            intersected_points_ind = 1:pt_size;
            intersected_points_ind(ray_A_val_ind) = 0; intersected_points_ind(ray_B_val_ind) = 0;
            
            intersect_ind = ray_A_val <= range_B(:,2) & ray_A_val >= range_B(:,1) & ray_B_val <= range_A(2) &  ray_B_val >= range_A(1);
            intersected_points = zeros(sum(intersect_ind),pt_size);
            intersected_points = [ray_A_val*ones(sum(intersect_ind),1),ray_B_val(intersect_ind)];
            
            intersected_points(:,ray_A_val_ind) = ray_A_val*ones(sum(intersect_ind),1);
            intersected_points(:,ray_B_val_ind) = ray_B_val(intersect_ind);
            intersected_points(:,boolean(intersected_points_ind)) = repelem(plane,sum(intersect_ind),1);
            min_dist = min([min(abs(range_B(intersect_ind,:) - ray_A_val)');min(abs(ray_B_val(intersect_ind) - range_A)')])';
            intersected_node_num = co_plannar_node_num(intersect_ind);
        end                                                            
        
        % function to create the graph_rep variable for point representation
        function graph_rep = create_point_graph_rep(obj)
            node_list = obj.point_node.node_list;
%             rounding_digit = 4;
            graph_rep = [];
            for k = 1:size(node_list,1)
                for i = 1:size(obj.grid.dim_disc_ia,1)
                    dimension_index = obj.grid.dim_disc_ia(i);
                    delta_q_vector = zeros(1,obj.model.numDofs);
                    delta_q = round(obj.grid.delta_q(dimension_index),obj.tolerance);
                    delta_q_vector(obj.grid.dim_disc_ia(i)) = delta_q;
                    for j = 1:2
                        if j == 1
                            nearby_point = node_list(k,2:obj.model.numDofs+1) + delta_q_vector;
                        else
                            nearby_point = node_list(k,2:obj.model.numDofs+1) - delta_q_vector;
                        end
                        %                         neighbour_point_index = find(ismember(node_list(:,2:obj.model.numDofs+1),round(nearby_point,rounding_digit),'rows'));
                        neighbour_point_index =  find(vecnorm((node_list(:,2:end)-nearby_point)')'<=10^-obj.tolerance);
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
%             obj.point_node(:).graph_rep =  graph_rep;
        end
        
        % function to create the node_list variable for ray representation
        function node_list = create_ray_node_list(obj)
            CASPR_log.Info('Creating ray workspace ray node list.');
            % covert the input conditions to same format as the ray
            % condition
            number_node = 0;
            node_list = cell(1,obj.model.numDofs+2);
            
            for i = 1:size(obj.rays, 1)
                % Determine the ray that has same condtions as input
                if(~isempty(obj.rays{i}))
                    interval_num = size(obj.rays{i}.intervals,1);
                    for ii = 1:interval_num
                    number_node = number_node + 1;  
                    node_list{number_node,1} = number_node; % node number
                    node_list{number_node,2} = obj.rays{i}.freeVariableIndex;
                    kk = 1;
                    for k = 3:size(node_list,2)
                        if k ~= obj.rays{i}.freeVariableIndex+2
                            node_list{number_node,k} =  obj.rays{i}.fixedVariables(kk);
                            kk = kk + 1;
                        else
                            node_list{number_node,k} =  obj.rays{i}.intervals(ii,:);
                        end
                    end
                    
                end
                end
                obj.ray_node(:).node_list = node_list;
            end
        end
        
        % function to create the graph_rep variable for ray representation
        function [graph_rep,intersected_point_set] = create_ray_graph_rep(obj, node_list, metrics)
            graph_rep = [];
            metric_value = [];
            point_node_list = [];
            ternimal_index = find(ismember([node_list{:,2}],node_list{end,2}));
            ternimal_index = ternimal_index(1);
            rays_seg = node_list;
            %%%%%
            for cur_ind = 1:size(rays_seg,1) - ternimal_index
                intersected_pt_order = zeros(1,obj.grid.n_dimensions);
                ray_A_node_num = rays_seg{cur_ind,1};
                ray_A = rays_seg(cur_ind,3:end);
                ray_A_free_val = rays_seg{cur_ind,2};
                intersected_pt_order(ray_A_free_val) = 1;
                if  obj.model.bodyModel.numDofs >2
                    for j = rays_seg{cur_ind,2}+1:obj.grid.n_dimensions
                        
                        ray_B_free_val = j;
                        intersected_pt_order(ray_B_free_val) = 1;
                        compare_ray_set = rays_seg([rays_seg{:,2}] == j,3:end);
                        if ~isempty(compare_ray_set)
                            ray_B_set_node_num = rays_seg([rays_seg{:,2}] == j,1);
                            [co_plannar_node_num,co_plannar_ind,plane] = obj.co_plannar_search(ray_A,compare_ray_set,rays_seg{cur_ind,2},j,ray_B_set_node_num);
                            [intersected_points,min_dist,intersected_node_num]  = obj.find_intersection(ray_A{ray_B_free_val},ray_A{ray_A_free_val},compare_ray_set(co_plannar_ind,ray_A_free_val),compare_ray_set(co_plannar_ind,j),...
                                ray_B_free_val,ray_A_free_val,plane,obj.grid.n_dimensions,co_plannar_node_num);
                            if ~isempty(intersected_points)
                                point_node_list = [point_node_list;intersected_points];
                                graph_rep  = [graph_rep;ones(size(intersected_node_num))*ray_A_node_num,intersected_node_num,min_dist];
                                if ~isempty(metrics)                                    
                                    for kk = 1:size(intersected_points,1)
                                        for k = 1:size(metrics,2)
                                            point_metric = PointWorkspaceElement(intersected_points(kk,:), obj.model, [], metrics(k));
                                            metric_value(kk,k) = point_metric.metrics{2};
                                        end
                                    end
                                  graph_rep = [graph_rep,  metric_value];
                                end
                            end
                        end
                    end
                end
            end
            intersected_point_set = point_node_list;
        end
        
        % function to check intersection  and distance between two co-plannar rays
        function [intersected_point,min_dist] = check_intersection(~,ray_1,ray_2)
            % check if co-planar
            for i = 1:size(ray_2,2)-1
                if i == ray_2{end} || i == ray_1{end}
                    intersected_point(i) = Inf;
                    co_planar_flag = 1;
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
            
            % find intersected point
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
        function graph = plot2ColorGraph(~,G,node_color_matrix,edge_color_matrix,title_str)
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
        function graph = plot1ColorGraph(~,G,node_color,edge_color,title_str)
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
    end
end
