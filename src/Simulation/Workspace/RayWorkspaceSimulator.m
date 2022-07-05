% The simulator to run a workspace simulation for rays
%
% Author        : Jonathan EDEN
% Created       : 2017
% Modified      : Paul 2022
% Description   :
%   Workspace simulator generates the workspace over a defined set of space
%   (currently only a grid of states is accepted). The workspace is created
%   using the ray method
classdef RayWorkspaceSimulator < SimulatorBase
    
    properties
        grid                    % Grid object for brute force workspace (input)
        workspace               % Final Workspace (output)
    end
    
    properties (SetAccess = private)
        compTime                    % Time to just evaluate the workspace
        offset = 0.01;          % offset of each ray interval
        free_variable_length    % The number of rays for each free variable
        conditions = []         % A list of conditions to be evaluated for
        metrics = []            % A list of metrics to be evaluated for
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = RayWorkspaceSimulator(model, grid, conditions, metrics,offset)
            w@SimulatorBase(model);
            w.grid          = grid;
            w.conditions    = conditions;
            w.metrics       = metrics;
            if ~isempty(offset)
                w.offset        = offset;
            end
        end
        
        
        % Implementation of the run function
        function run(obj,varargin)
            w = warning ('off','all');
            obj.compTime = 0;
            % Test if the metrics have infinite limits
            for i = 1:size(obj.metrics,2)
                if((~isempty(obj.metrics{i}.metricMax))&&((abs(obj.metrics{i}.metricMax)==Inf)||(abs(obj.metrics{i}.metricMin)==Inf)))
                    CASPR_log.Warn('A metric with infinite limit values cannot be plotted. To plot please set the metric limit to be finite or filter the workspace after plotting');
                end
            end
            GenNodeList = 1;
            
            if ~isempty(varargin)
                for i = 1:2:size(varargin,2)
                    if strcmpi(varargin{i},'GenerateNodeList')
                        GenNodeList = varargin{i};
                        
                    end
                end
            end
            
            n_grid_points = 0;
            
            for i =1:size(obj.grid.dim_disc_ia)
                grid_index = true(obj.grid.n_dimensions,1); grid_index(i) = false;
                obj.free_variable_length(i) = prod(obj.grid.q_length(grid_index));
                n_grid_points = n_grid_points + obj.free_variable_length(i);
            end
            
            workspace_count = 0;
            obj.workspace = RayWorkspace(obj.model, obj.grid);
            if GenNodeList
                for i = 1:obj.grid.n_points
                    obj.workspace.point_node.node_list(i,:) = [i,obj.grid.getGridPoint(i)'];
                end
                node_list = obj.workspace.point_node.node_list(:,2:end);
                node_list_num = obj.workspace.point_node.node_list(:,1);
            end
            if ~isempty(obj.grid.connectivity_list)
                obj.workspace.point_node.graph_rep = obj.grid.connectivity_list;
                
                graph_rep = obj.workspace.point_node.graph_rep;
            end
            % Runs over each dimension and construct the rays for that
            % dimension
            % each point
            k = 1;
            
            log_level = CASPRLogLevel.DEBUG;
            is_log = (log_level >= CASPR_log.GetLogLevel());
            start_time_i = clock;
            tic;
            dimension_array = 1:obj.grid.n_dimensions;
            save_file_idx = zeros(obj.grid.n_dimensions,1);
            save_path = [CASPR_configuration.LoadHomePath,'\data\AutoGenScripts\',obj.model.robotName,'\'];
            if ~exist(save_path, 'dir')
                mkdir(save_path);
            end
            addpath(save_path);
            
            save_file_names = [];
            save_file_count = 0;
            save_file_id = 0;
            
            node_handle_time = 0;
            
            node_offset = zeros(size(obj.grid.q_length));
            node_offset(1) = 1;
            step_size = flipud(obj.grid.q_length - 1);
            for i = 2:numel(obj.grid.q_length)
                node_offset(i) =  (step_size(i-1)+1) *  node_offset(i-1);
            end
            node_offset = flipud(node_offset);
            infeasible_node_array_index = cell(obj.grid.n_points ,1);
            new_node_list = cell(obj.grid.n_points ,1);
            new_graph_rep = cell(obj.grid.n_points ,1);
            current_max_node_num = size(obj.workspace.point_node.node_list,1);
            cell_count = 0;
            ray_to_save = [];
            tstart = tic;
            
            for i = 1:obj.grid.n_dimensions
                
                CASPR_log.Info(sprintf('Ray workspace analysis DoF %d.', i));
                
                if ismember(i, obj.grid.dim_disc_ia)
                    
                    %i is the free variable index;
                    grid_index = true(obj.grid.n_dimensions,1);
                    grid_index(i) = false;
                    % Create a subgrid
                    sub_grid = UniformGrid(obj.grid.q_begin(grid_index), obj.grid.q_end(grid_index), obj.grid.delta_q(grid_index), 'step_size', obj.grid.q_wrap(grid_index),0);
                    
                    start_time_j = fix(clock);
                    point_count = 0;
                    window_num = 0;
                    for j = 1:sub_grid.n_points
                        
                        if point_count == node_offset(i)
                            window_num = window_num + 1;
                            point_count = 0;
                        end
                        
                        same_ray_node_array_idx = j + (0:(obj.grid.q_length(i)-1))*node_offset(i) + window_num*(obj.grid.q_length(i)-1)*node_offset(i);
                        
                        
                        if (is_log)
                            CASPR_log.Print(sprintf('Workspace DoF %d. Workspace ray %d. Completion Percentage: %3.2f.', i, j, 100*k/n_grid_points), log_level);
                        end
                        % Load the current fixed grid coordinates
                        q_fixed = sub_grid.getGridPoint(j);
                        
                        if (obj.grid.q_begin(i) ~= obj.grid.q_end(i))
                            % Construct the workspace ray
                            wre = RayWorkspaceElement(obj.model, q_fixed, obj.conditions, i, [obj.grid.q_begin(i),obj.grid.q_end(i)],obj.offset);
                            obj.compTime = obj.compTime + wre.compTime;
                            
                            if ~isempty(wre.intervals)
                                workspace_count = workspace_count + 1;
                                save_file_count = save_file_count + 1;
                                ray_to_save{save_file_count}  = wre;
                                
                                if GenNodeList
                                    currentInterval = wre.intervals;                                    
                                    %% update the node list, add new nodes and record the removed node index
                                    node_handle_time_count = tic;
                                    currentInterval = wre.intervals;
                                    related_points = node_list(same_ray_node_array_idx,i);
                                    isInInterval = [];
                                    for II = 1:size(currentInterval,1)
                                        isInInterval(:,II) = currentInterval(II,1) <= related_points & currentInterval(II,2) >= related_points;
                                    end
                                    
                                    new_point_idx = ~ismember(currentInterval,related_points);
                                    short_interval_idx = find(sum(new_point_idx,2)>0 & ~logical(sum(isInInterval))');
                                    long_interval_idx = find(sum(new_point_idx,2)>0 & logical(sum(isInInterval))');
                                    cell_count = cell_count + 1;
                                    
                                    long_interval = currentInterval(long_interval_idx,:);
                                    short_interval = currentInterval(short_interval_idx,:);
                                    
                                    long_new_point_idx = ~ismember(long_interval,related_points);
                                    short_new_point_idx = ~ismember(short_interval,related_points);
                                    
                                    long_interval_points = sort(long_interval(long_new_point_idx));
                                    short_interval_points = sort(short_interval(short_new_point_idx));
                                    added_graph_rep = [];added_node_list = [];
                                    
                                    if ~isempty(long_interval_points)
                                        
                                        q_tmp = repmat(obj.model.q',numel(long_interval_points),1);
                                        q_tmp(:,i) = long_interval_points(:);
                                        q_tmp(:,grid_index) = repmat(q_fixed',numel(long_interval_points),1);
                                        new_node_num = current_max_node_num + (1:numel(long_interval_points));
                                        added_node_list = [added_node_list;[new_node_num(:),q_tmp]];
                                        
                                        current_max_node_num = new_node_num(end);
                                        
                                        stillInRangePoint = related_points(logical(sum(isInInterval,2)));
                                        
                                        if ~isempty(stillInRangePoint)
                                            stillInRangeNodeNum = same_ray_node_array_idx(logical(sum(isInInterval,2)));
                                            
                                            
                                            for ni = 1:numel(long_interval_points)
                                                [dist,min_idx] = min(abs(long_interval_points(ni) - stillInRangePoint));
                                                added_graph_rep = [added_graph_rep;
                                                    new_node_num(ni),stillInRangeNodeNum(min_idx),dist];
                                            end
                                            
                                            
                                        end
                                    end
                                    if  ~isempty(short_interval_points)
                                        q_tmp = repmat(obj.model.q',numel(short_interval_points),1);
                                        q_tmp(:,i) = short_interval_points(:);
                                        q_tmp(:,grid_index) = repmat(q_fixed',numel(short_interval_points),1);
                                        new_node_num = current_max_node_num + (1:numel(short_interval_points));
                                        added_node_list = [added_node_list;[new_node_num(:),q_tmp]];
                                        
                                        
                                        current_max_node_num = new_node_num(end);
                                        for ni = 1:2:numel(short_interval_points)
                                            added_graph_rep = [added_graph_rep;
                                                new_node_num(ni),new_node_num(ni+1),short_interval_points(ni+1)-short_interval_points(ni)];
                                        end
                                        
                                        
                                    end
                                    new_node_list{cell_count} = added_node_list;
                                    new_graph_rep{cell_count} = added_graph_rep;
                                    
                                    infeasible_node_array_index{cell_count} = same_ray_node_array_idx(~logical(sum(isInInterval,2)));
                                    node_handle_time = node_handle_time + toc(node_handle_time_count);
                                end
                            else
                                if GenNodeList
                                    node_handle_time_count = tic;
                                    cell_count = cell_count + 1;
                                    infeasible_node_array_index{cell_count} = same_ray_node_array_idx;
                                    node_handle_time = node_handle_time + toc(node_handle_time_count);
                                end
                            end
                            
                            k = k+1;
                            %% time estimating
                            time_diff_j = etime(clock,start_time_j);
                            est_t_j = sub_grid.n_points * (time_diff_j/j);
                            end_toc = toc(tstart);
                            if  end_toc >= 5
                                tstart = tic;
                                disp(['Etsimate time needed for the ',num2str(i), ' DoF =',num2str(est_t_j,'%4.1f'),'sec' ]);
                                dt = datetime(start_time_j,'InputFormat', 'HH:mm:ss');
                                dt.Format = 'eeee, MMMM d, yyyy h:mm:ss a';
                                disp(['Etsimate finish time: ',char( dt + seconds(est_t_j))]);
                            end
                            %% save data for saving time
                            if save_file_count >= 3000
                                save_file_id = save_file_id + 1;
                                save_name = sprintf('%d_%d_%d_%d_%d_Ray_Data_Dof%d_Set%d.mat',[start_time_i(1:end-1),i,save_file_id]);
                                save_file_names{save_file_id} = save_name;
                                save(fullfile(save_path,save_name),'ray_to_save');
                                ray_to_save = [];
                                save_file_count = 0;
                            end
                        end
                        point_count = point_count + 1;
                    end
                    
                    
                    %% time estimating
                    time_diff_i = etime(clock,start_time_i);
                    est_t_i = obj.grid.n_dimensions  * (time_diff_i/i);
                    disp(['Etsimate time needed for entire workspace =',num2str(est_t_i,'%4.1f'),'sec' ]);
                    dt = datetime(start_time_i,'InputFormat', 'HH:mm:ss');
                    dt.Format = 'eeee, MMMM d, yyyy h:mm:ss a';
                    disp(['Etsimate finish time: ',char( dt + seconds(est_t_i))]);
                    %                     end
                end
                
            end

            obj.workspace.rays = [];
            for file_id = 1:save_file_id
                load_name = save_file_names{file_id};
                loaded_data = load(load_name);
                obj.workspace.rays = [obj.workspace.rays;loaded_data.ray_to_save'];
                delete([save_path,load_name]);
            end
            obj.workspace.rays = [obj.workspace.rays;ray_to_save'];
            if GenNodeList
                
                disp('Updating the nodes...');
                
                node_handle_time_count = tic;
                node_list = obj.workspace.point_node.node_list(:,2:end);
                node_list_num = obj.workspace.point_node.node_list(:,1);
                graph_rep = obj.workspace.point_node.graph_rep;
                
                new_node_list = cell2mat(new_node_list);
                if ~isempty(new_node_list)
                    node_list_num = [node_list_num;new_node_list(:,1)];
                    node_list = [node_list;new_node_list(:,2:end)];
                end
                
                new_graph_rep = cell2mat(new_graph_rep);
                graph_rep = [graph_rep;new_graph_rep];
                
                
                remove_array_num =unique([infeasible_node_array_index{:}]);
                
                
                
                arrayIdx1 = find(ismember(graph_rep(:,1),remove_array_num));
                arrayIdx2 = find(ismember(graph_rep(:,2),remove_array_num));
                arrayIdx = unique([arrayIdx1;arrayIdx2]);
                graph_rep(arrayIdx,:) = [];
                
                % re-assign node number in graph
                tmp_graph_rep_1 = sort(graph_rep(:,[1,2]),2);
                [~,index] = sortrows(tmp_graph_rep_1,1);
                tmp_graph_rep_2 = graph_rep(index,:);
                tmp_graph_rep_2 = [sort(tmp_graph_rep_2(:,[1,2]),2),tmp_graph_rep_2(:,3)];
                G = graph(tmp_graph_rep_2(:,1),tmp_graph_rep_2(:,2),tmp_graph_rep_2(:,3));
                deg = degree(G);
                zero_deg = find(deg == 0);
                
                node_list(zero_deg,:) = [];
                node_list_num(zero_deg,:) = [];
                
                H = rmnode(G,zero_deg);
                graph_rep = H.Edges;
                replaceNodeNum = (1:numel(node_list_num))';
                obj.workspace.point_node.node_list = [replaceNodeNum,node_list];
                obj.workspace.point_node.graph_rep = graph_rep;
                node_handle_time = node_handle_time + toc(node_handle_time_count);
            end
            warning ('on','all');
        end
        
        
    end
    methods(Access = private)
        
    end
end
