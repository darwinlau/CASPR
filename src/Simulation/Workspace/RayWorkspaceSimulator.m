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
        grid                    % Grid object for brute force workspace (input)
        free_variable_length    % The number of rays for each free variable
        workspace               % Final Workspace (output)
        conditions = []         % A list of conditions to be evaluated for
        metrics = []            % A list of metrics to be evaluated for
        options                 % The workspace simulator options
        graph_rep = []          % The graph representation for the workspace
        node_list = []          % A list of all nodes
        comp_time               % Computational time structure
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = RayWorkspaceSimulator(model,grid,options)
            w@SimulatorBase(model);
            w.grid          = grid;
            w.options       = options;
            w.comp_time     = struct('ray_construction',0,'graph_construction',0,'total',0);
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
                if isempty(obj.options.slices) % all elements in q would be discretized
                    n_grid_points = 0; 
                    for i =1:obj.grid.n_dimensions
                        grid_index = true(obj.grid.n_dimensions,1); grid_index(i) = false;
%                     for i =1:length(obj.dim_disc_ia)
%                         grid_index = true(obj.grid.n_dimensions,1); grid_index(obj.(i)) = false;
                        obj.free_variable_length(i) = prod(obj.grid.q_length(grid_index));
                        n_grid_points = n_grid_points + obj.free_variable_length(i);
                    end 
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
                    ray_t_in = tic;
                    total_t_in = tic;
                    for i = 1:obj.grid.n_dimensions
                        grid_index = true(obj.grid.n_dimensions,1); grid_index(i) = false;
                        % Create a subgrid
                        sub_grid = UniformGrid(obj.grid.q_begin(grid_index),obj.grid.q_end(grid_index),obj.grid.delta_q(grid_index),'step_size',obj.grid.q_wrap(grid_index));
                        for j = 1:sub_grid.n_points
    %                         CASPR_log.Info([sprintf('Workspace ray %d. ',k),sprintf('Completion Percentage: %3.2f',100*k/n_grid_points)]);
                            % Load the current fixed grid coordinates
                            q_fixed = sub_grid.getGridPoint(j);
                            % Construct the workspace ray
                            wr = RayWorkspaceElement(q_fixed,n_metrics,n_conditions,i,[obj.grid.q_begin(i),obj.grid.q_end(i)]);
                            
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
                else % users have specified the slices to be investigated
                    % NOTE the deg. specified by users, then they cannot be
                    % selected as free variable of poly. eqs. in Ray-Based method
                    % NOTE the deg. that are not specified are called free_deg_candidate
                    
                    % indices of the degs. specified by users (A.K.A. con_deg) (e.g. x i.e., index=[1]) 
                    con_deg_indices = zeros(size(obj.options.slices,1),1);
                    % indices of the slices. specified by users (e.g. [4]th slices w.r.t. x)
                    con_slc_indices=cell(size(obj.options.slices,1),1);
                    % values of the specific slices (e.g. x=0+0.2*(4-1)=[0.6])
                    con_slc_val=cell(size(obj.options.slices,1),1);
                    % num. of the slices of EACH specific degree (e.g. num=[1] one slice for x)
                    num_slc_perDeg = zeros(size(obj.options.slices,1),1);
                    % logical values to present free_deg_candidate(1) and
                    % con_deg(i.e., specific deg.)(0) (e.g. [0 1 1]^T for a 3D translation CDPR)
                    deg_lgc =true(obj.grid.n_dimensions,1);
                    for i = 1:size(obj.options.slices,1)
                        CASPR_log.Assert(obj.options.slices{i,1}<=obj.grid.n_dimensions,'Invlaid Degrees Specified by Users');
                        CASPR_log.Assert(size(obj.options.slices{i,2},1)==1,'The Slices to be Investigated Are in the Wrong Form');
                        CASPR_log.Assert(max(obj.options.slices{i,2})<=obj.grid.q_length(obj.options.slices{i,1}),'At Least One of the Specified Slices is Invalid');
                        con_deg_indices(i) = obj.options.slices{i,1};
                        deg_lgc(con_deg_indices(i)) = false; %1 for free_var; 0 for con_var
                        con_slc_indices{i} = obj.options.slices{i,2};
                        con_slc_val{i} = obj.grid.q_begin(con_deg_indices(i))*ones(size(con_slc_indices{i}))+...
                                            obj.grid.delta_q(con_deg_indices(i))*(con_slc_indices{i}-1);
                        num_slc_perDeg(i) = length(obj.options.slices{i,2});
                    end
                    
                    % To calculate the num. of grid pts
                    % - the num_pt from con_deg (product of the num. of slices per con_deg)
                    num_slices = prod(num_slc_perDeg);
                    % the num_pt w.r.t all free_deg_candidate
                    num_freeVar = 0;
                    % the num_pt w.r.t one of free_deg_candidate
                    free_var_q_length = obj.grid.q_length(deg_lgc);
%                     for i =1:obj.grid.n_dimensions
%                         grid_index = true(obj.grid.n_dimensions,1); grid_index(i) = false;
%                     for i =1:length(obj.dim_disc_ia)
%                         grid_index = true(obj.grid.n_dimensions,1); grid_index(obj.dim_disc_ia(i)) = false;
                    % - num_pt from free_deg_candidate
                    for i = 1: sum(deg_lgc) % up to all free_deg_candidate
                        grid_index = true(sum(deg_lgc),1);
                        grid_index(i) = false;
                        obj.free_variable_length(i) = prod(free_var_q_length(grid_index));
                        num_freeVar = num_freeVar + obj.free_variable_length(i);
                    end 
                    % n_grid_points = (num_pt from free_deg_candidate)*(num_pt from con_deg)
                    n_grid_points = num_freeVar*num_slices;
                    
                    obj.workspace = cell(n_grid_points,1);
                    workspace_count = 0; % num. of non-null ws/grid_points
                    n_metrics       = length(obj.metrics);
                    n_conditions    = length(obj.conditions);
                    % Determine translation from workspace_in to current metrics
                    % list            

                    % Runs over each dimension and construct the rays for that
                    % dimension
                    % each point
                    k = 1;
                    ray_t_in = tic;
                    total_t_in = tic;
                    
                    % For each combination of these specific slices
    %                 for i = 1:obj.grid.n_dimensions
    %                     grid_index = true(obj.grid.n_dimensions,1); grid_index(i) = false;
                    for cnnt = 1:num_slices
                        % Determine values of one of combinations between these con_slc
                        con_slc_val_a = zeros(size(con_deg_indices));
                        remainder = cnnt;
                        for k = 1:length(con_slc_val_a)
                            if remainder~=0
                                pp = ceil(remainder/prod(num_slc_perDeg(k+1:end)));
                                con_slc_val_a(k)= con_slc_val{k}(pp);
                            else
                                con_slc_val_a(k)= con_slc_val{k}(end);
                            end                            
                            remainder = rem(remainder,prod(num_slc_perDeg(k+1:end)));
                        end
                        % NOTE the degs. whose min=max would not be discretized by UniformGrid
                        % (i.e., q_length=1 for the degs. whose min=max by UniformGrid)
                        % To set min=max of con_deg
                        llower = obj.grid.q_begin; uuper = obj.grid.q_end;
                        llower(con_deg_indices) = con_slc_val_a;
                        uuper(con_deg_indices) = con_slc_val_a;
                        % indices of free_deg_candidate 
                        % (i.e., indices of the dims to be discretized)
                        dim_disc_ia = find(deg_lgc==1);
                        for i = 1:length(dim_disc_ia)
                            free_var_i = dim_disc_ia(i);
                            grid_index = true(obj.grid.n_dimensions,1); grid_index(free_var_i) = false;
                            % Create a subgrid 
                            sub_grid = UniformGrid(llower(grid_index),uuper(grid_index),obj.grid.delta_q(grid_index),'step_size',obj.grid.q_wrap(grid_index));
                            for j = 1:sub_grid.n_points
        %                         CASPR_log.Info([sprintf('Workspace ray %d. ',k),sprintf('Completion Percentage: %3.2f',100*k/n_grid_points)]);
                                % Load the current fixed grid coordinates
                                q_fixed = sub_grid.getGridPoint(j);
                                % Construct the workspace ray
        %                         wr = RayWorkspaceElement(q_fixed,n_metrics,n_conditions,i,[obj.grid.q_begin(i),obj.grid.q_end(i)]);
                                wr = RayWorkspaceElement(q_fixed,n_metrics,n_conditions,free_var_i,[obj.grid.q_begin(free_var_i),obj.grid.q_end(free_var_i)]);

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
                    end
                end
                obj.comp_time.ray_construction = toc(ray_t_in);
                graph_t_in = tic;
                if((nargin == 2) || isempty(w_metrics))
                    obj.createWorkspaceGraph(w_conditions{1},[]);
                else
                    obj.createWorkspaceGraph(w_conditions{1},w_metrics{1});
                end
                obj.comp_time.graph_construction = toc(graph_t_in);
                obj.comp_time.total = toc(total_t_in);
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
                            %%% results could be multiple intervals
                            axis1=obj.workspace{it}.conditions{2}';%':respect "plot" fn
                            axis2=consvar(plot_axis(2))*ones(size(axis1));
                            axis3=consvar(plot_axis(3))*ones(size(axis1));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(2)
                            axis2=obj.workspace{it}.conditions{2}';
                            axis1=consvar(plot_axis(1))*ones(size(axis2));
                            axis3=consvar(plot_axis(3))*ones(size(axis2));
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(3)
                            axis3=obj.workspace{it}.conditions{2}';
                            axis1=consvar(plot_axis(1))*ones(size(axis3));
                            axis2=consvar(plot_axis(2))*ones(size(axis3));                            
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
                            axis1=obj.workspace{it}.conditions{2}';
                            axis2=consvar(plot_axis(2))*ones(size(axis1)); 
                            plotflag=1;
                        elseif obj.workspace{it}.free_variable_index==plot_axis(2)
                            axis2=obj.workspace{it}.conditions{2}';
                            axis1=consvar(plot_axis(1))*ones(size(axis2));  
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
                        if(obj.node_list(j,3+obj.grid.n_dimensions)==i)
                            G.Nodes.NodeColours(j) = -edge_range/(max_list-min_list)*(centroid_list(j) - min_list) + min(G.Edges.EdgeColours) - edge_range/127.99;
                        else
                            G.Nodes.NodeColours(j) = zero_colour;
                        end
                    end
                    p.NodeCData = G.Nodes.NodeColours;
                elseif(i == obj.grid.n_dimensions+1)
                    ray_distance_list = zeros(G.numnodes,1);
                    for j = 1:G.numnodes
                        ray_distance_list(j) =  (obj.node_list(j,3) - obj.node_list(j,2))/(obj.grid.q_end(obj.node_list(j,3+obj.grid.n_dimensions)) - obj.grid.q_begin(obj.node_list(j,3+obj.grid.n_dimensions)));
                    end
                    max_list = max(ray_distance_list);
                    min_list = min(ray_distance_list);
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
                        G.Nodes.NodeColours(j) = -edge_range/(max_list-min_list)*(ray_distance_list(j) - min_list) + min(G.Edges.EdgeColours) - edge_range/128;
                    end
                    p.NodeCData = G.Nodes.NodeColours;
                else
                    p.NodeCData = (zero_colour-0.1)*ones(G.numnodes,1);
                end
                if(i <= obj.grid.n_dimensions+1)
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
                    p.EdgeCData = G.Edges.EdgeColours;
                    p.LineWidth = 2;
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
            obj.node_list = zeros(10*number_rays,3+obj.grid.n_dimensions); 
            number_node = 0;
            % For each ray
            for i = 1:number_rays
                % Determine the number of the condition
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
            
            % Computation for the maximum number of nodes
%             max_edges = 0;
%             for i = 1:obj.grid.n_dimensions
%                 % Determine the number of nodes for a given free variable
%                 % index
%                 num_nodes_i = sum(obj.node_list(:,3+obj.grid.n_dimensions)==i);
%                 max_edges = max_edges + num_nodes_i*(number_node-num_nodes_i);
%             end
            max_edges = number_node*max(obj.grid.q_length)*obj.grid.n_dimensions;

            % Initialise an adjacency list
            obj.graph_rep = zeros(max_edges,2+obj.grid.n_dimensions+1+metric_flag);
            number_intersects = 0;
            fixed_index_intersect_candidate = zeros(obj.grid.n_dimensions-1,1);
            intersect_fixed_indices = zeros(obj.grid.n_dimensions-1,1);
            for i = 1:number_node
                % Determine the current workspace index and interval
                workspace_index_i = obj.node_list(i,1);
                workspace_interval_i = obj.node_list(i,2:3);
                free_variable_index = obj.workspace{workspace_index_i}.free_variable_index;
                if(free_variable_index == obj.grid.n_dimensions)
                    % There are no possible intersects that haven't been
                    % already checked
                    break;
                else
                    active_vector = false(obj.grid.n_dimensions-1,1); active_vector(free_variable_index) = true;
                    fixed_indices = obj.workspace_index_to_index_vector(workspace_index_i,free_variable_index);
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
                        if(~isempty(obj.workspace{j_index}))
                            % find the node list entry
                            node_indices = find(obj.node_list(:,1) == j_index);
                            for k = 1:length(node_indices)
                                [is_intersected,intersection_point] = obj.workspace{workspace_index_i}.intersect(workspace_interval_i,obj.workspace{obj.node_list(node_indices(k),1)},obj.node_list(node_indices(k),2:3));
                                if(is_intersected)
                                    number_intersects = number_intersects + 1;
                                    min_dist = min([abs(intersection_point(obj.node_list(i,3+obj.grid.n_dimensions))-obj.node_list(i,2)),abs(intersection_point(obj.node_list(i,3+obj.grid.n_dimensions))-obj.node_list(i,3)),abs(intersection_point(obj.node_list(node_indices(k),3+obj.grid.n_dimensions))-obj.node_list(node_indices(k),2)),abs(intersection_point(obj.node_list(node_indices(k),3+obj.grid.n_dimensions))-obj.node_list(node_indices(k),3))]);
                                    % Determine the fixed_indices
                                    free_variable_index_j = obj.workspace{j_index}.free_variable_index;
                                    fixed_indices_j = obj.workspace_index_to_index_vector(j_index,free_variable_index_j);
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
                                    if(metric_flag)
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
