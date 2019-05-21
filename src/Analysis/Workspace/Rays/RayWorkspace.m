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
        ray_node = []             % use intersected point as node
        graph_rep = []              % The graph representation for the workspace
        node_list = []              % A list of all nodes
    end
    
    methods
        function pw = RayWorkspace(model, grid)
            pw.model = model;
            pw.grid = grid;
            pw.rays = cell(grid.n_points,1);
        end
        
        function createWorkspaceGraph(obj,condition,metric)
            if(isempty(metric))
                num_metrics = 0;
            else
                num_metrics = 1;
            end
            
            % For the moment this will be written as if there was only one
            % condition I will subsequently modify
            number_rays = length(obj.rays);
            % Create list of nodes
            obj.node_list = zeros(10*number_rays,3+obj.grid.n_dimensions);
            number_node = 0;
            % For each ray
            for i = 1:number_rays
                % Determine the number of the condition
                if(~isempty(obj.rays{i}))
                    n_constraints = size(obj.rays{i}.conditions,1);
                    intervals = [];
                    for j = 1:n_constraints
                        if(condition.type == obj.rays{i}.conditions{j,1})
                            intervals = obj.rays{i}.conditions{j,2};
                            break;
                        end
                    end
                    for j = 1:size(intervals,1)
                        number_node = number_node + 1;
                        obj.node_list(number_node,:) = [i,intervals(j,:),obj.rays{i}.fixed_variables',obj.rays{i}.free_variable_index];
                    end
                end
            end
            % Resize to the correct size
            obj.node_list = obj.node_list(1:number_node,:);
            
            % Computation for the maximum number of edges
            max_edges = number_node*max(obj.grid.q_length)*obj.grid.n_dimensions;
            
            % Initialise an adjacency list
            obj.graph_rep = zeros(max_edges,2+obj.grid.n_dimensions+1+num_metrics);
            number_intersects = 0;
            fixed_index_intersect_candidate = zeros(obj.grid.n_dimensions-1,1);
            intersect_fixed_indices = zeros(obj.grid.n_dimensions-1,1);
            for i = 1:number_node
                % Determine the current workspace index and interval
                workspace_index_i = obj.node_list(i,1);
                workspace_interval_i = obj.node_list(i,2:3);
                free_variable_index = obj.rays{workspace_index_i}.free_variable_index;
                if(free_variable_index == obj.grid.n_dimensions)
                    % There are no possible intersects that haven't been
                    % already checked
                    break;
                else
                    active_vector = false(obj.grid.n_dimensions-1,1); active_vector(free_variable_index) = true;
                    fixed_indices = obj.rays_index_to_index_vector(workspace_index_i,free_variable_index);
                    % Now remove the indices that are not coplanar
                    % For each subsequent dimension determine the indices
                    % Possible sources of intersect
                    coplanar_indices = zeros((obj.grid.n_dimensions-free_variable_index)*obj.grid.q_length(free_variable_index),1);
                    current_coplanar_index = 1;
                    for dimension_index = free_variable_index+1:obj.grid.n_dimensions
                        % First determine the fixed index components
                        fixed_index_nm2 = fixed_indices;
                        % Remove the value for the new dimension
                        fixed_index_nm2(dimension_index-1) = [];
                        fixed_index_intersect_candidate(~active_vector) = fixed_index_nm2;
                        for growth_index = 1:obj.grid.q_length(free_variable_index)
                            fixed_index_intersect_candidate(active_vector) = growth_index;
                            % Convert the vector back into an index
                            coplanar_indices(current_coplanar_index) = obj.index_vector_to_workspace_index(fixed_index_intersect_candidate,dimension_index);
                            current_coplanar_index = current_coplanar_index + 1;
                        end
                    end
                    intersect_check_i = coplanar_indices;
                    for j = 1:length(intersect_check_i)
                        j_index = intersect_check_i(j);
                        if(~isempty(obj.rays{j_index}))
                            % find the node list entry
                            node_indices = find(obj.node_list(:,1) == j_index);
                            for k = 1:length(node_indices)
                                [is_intersected,intersection_point] = obj.rays{workspace_index_i}.intersect(workspace_interval_i,obj.rays{obj.node_list(node_indices(k),1)},obj.node_list(node_indices(k),2:3));
                                if(is_intersected)
                                    number_intersects = number_intersects + 1;
                                    min_dist = min([abs(intersection_point(obj.node_list(i,3+obj.grid.n_dimensions))-obj.node_list(i,2)),abs(intersection_point(obj.node_list(i,3+obj.grid.n_dimensions))-obj.node_list(i,3)),abs(intersection_point(obj.node_list(node_indices(k),3+obj.grid.n_dimensions))-obj.node_list(node_indices(k),2)),abs(intersection_point(obj.node_list(node_indices(k),3+obj.grid.n_dimensions))-obj.node_list(node_indices(k),3))]);
                                    % Determine the fixed_indices
                                    free_variable_index_j = obj.rays{j_index}.free_variable_index;
                                    fixed_indices_j = obj.rays_index_to_index_vector(j_index,free_variable_index_j);
                                    active_vector_i = false(obj.grid.n_dimensions,1); active_vector_i(free_variable_index) = true;
                                    active_vector_j = false(obj.grid.n_dimensions,1); active_vector_j(free_variable_index_j) = true;
                                    % Determine the fixed variable
                                    % values
                                    intersect_fixed_indices(~active_vector_i) = fixed_indices;
                                    intersect_fixed_indices(~active_vector_j) = fixed_indices_j;
                                    for intersection_dimension_index = 1:obj.grid.n_dimensions
                                        if((intersection_dimension_index ~= free_variable_index)&&(intersection_dimension_index ~= free_variable_index_j))
                                            % Convert the fixed variable
                                            % into an index
                                            intersect_fixed_temp = intersect_fixed_indices;
                                            intersect_fixed_temp(intersection_dimension_index) = [];
                                            % Convert the index into a node
                                            workspace_index_min_dist = obj.index_vector_to_workspace_index(intersect_fixed_temp,intersection_dimension_index);
                                            % list entry
                                            node_index_candidates = find(obj.node_list(:,1) == workspace_index_min_dist);
                                            % compute the distance
                                            for dist_index = 1:size(node_index_candidates,1)
                                                k_dist = min([abs(intersection_point(obj.node_list(node_index_candidates(dist_index),3+obj.grid.n_dimensions))-obj.node_list(node_index_candidates(dist_index),2)),abs(intersection_point(obj.node_list(node_index_candidates(dist_index),3+obj.grid.n_dimensions))-obj.node_list(node_index_candidates(dist_index),3))]);
                                                % Update the minimum if
                                                % necessary
                                                if(k_dist <= min_dist)
                                                    min_dist = k_dist;
                                                end
                                            end
                                        end
                                    end
                                    obj.graph_rep(number_intersects,1) = i;
                                    obj.graph_rep(number_intersects,2) = node_indices(k);
                                    obj.graph_rep(number_intersects,3:2+obj.grid.n_dimensions) = transpose(intersection_point);
                                    obj.graph_rep(number_intersects,2+obj.grid.n_dimensions+1) = min_dist;
                                    if(num_metrics)
                                        obj.model.update(intersection_point,zeros(obj.grid.n_dimensions,1),zeros(obj.grid.n_dimensions,1),zeros(obj.grid.n_dimensions,1));
                                        [~,obj.graph_rep(number_intersects,2+obj.grid.n_dimensions+2),~] = obj.metrics{1}.evaluate(obj.model,[]);
                                    end
                                end
                            end
                        end
                    end
                end
            end
            obj.graph_rep = obj.graph_rep(1:number_intersects,:);
        end
    end
    
    methods(Access = private)
        
        function index_vector = workspace_index_to_index_vector(obj,workspace_index,free_variable_index)
            index_vector = zeros(obj.grid.n_dimensions-1,1);
            temp_index = workspace_index; temp_index = temp_index - sum(obj.free_variable_length(1:free_variable_index-1));
            dimension_vector_temp = 1:obj.grid.n_dimensions;
            dimension_vector_temp(free_variable_index) = [];
            q_length_temp = obj.grid.q_length(dimension_vector_temp);
            q_div = prod(q_length_temp);
            for dimension_index = 1:obj.grid.n_dimensions-1
                q_div = q_div/q_length_temp(dimension_index);
                index_vector(dimension_index) = ceil((temp_index)/q_div);
                temp_index = temp_index - (index_vector(dimension_index)-1)*q_div;
            end
        end
        
        function workspace_index = index_vector_to_workspace_index(obj,index_vector,free_variable_index)
            dimension_vector_temp = 1:obj.grid.n_dimensions;
            dimension_vector_temp(free_variable_index) = [];
            q_length_temp = obj.grid.q_length(dimension_vector_temp);
            q_div = prod(q_length_temp);
            workspace_index = sum(obj.free_variable_length(1:free_variable_index-1))+1;
            for count_index = 1:obj.grid.n_dimensions-1
                q_div = q_div/q_length_temp(count_index);
                workspace_index = workspace_index + q_div*(index_vector(count_index)-1);
            end
        end
    end
    
end