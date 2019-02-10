% The simulator to run a workspace simulation
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
%   Workspace simulator generates the workspace over a defined set of space
%   (currently only a grid of states is accepted). The workspace simulation
%   performs essentially a numerical brute force approach over the set of
%   poses for a given workspace condition and/or metric.
classdef PointWorkspaceSimulator < SimulatorBase
    
    properties
        grid            % Grid object for brute force workspace (input)
        workspace       % Final Workspace (output)
        conditions = [] % A list of conditions to be evaluated for
        metrics = []    % A list of metrics to be evaluated for
        options         % The workspace simulator options
        edge_matrix = []% The graph representation for the workspace
        graph_rep = []  % The graph representation for the workspace
        node_list = []  % A list of all nodes
        comp_time       % Computational time structure
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = PointWorkspaceSimulator(model,grid,options)
            w@SimulatorBase(model);
            w.grid          = grid;
            w.options       = options;
            w.comp_time     = struct('point_construction',0,'graph_construction',0,'total',0);
        end
        
        % Implementation of the run function
        function run(obj, w_conditions, w_metrics, w_connectivity)
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
            n_grid_points = obj.grid.n_points;
            obj.workspace = cell(n_grid_points,1);
            workspace_count = 0;
            n_metrics       = length(obj.metrics);
            n_conditions    = length(obj.conditions);
            % Determine translation from workspace_in to current metrics
            % list            
            
            % Timing variables
            point_t_in = tic;
            total_t_in = tic;
            
            % Runs over the grid and evaluates the workspace condition at
            % each point
            for i = 1:n_grid_points
                CASPR_log.Print([sprintf('Workspace point %d. ',i),sprintf('Completion Percentage: %3.2f',100*i/n_grid_points)],CASPRLogLevel.INFO);
                % Get the grid point
                q = obj.grid.getGridPoint(i);
                % Construct the workspace point
                wp = PointWorkspaceElement(q,n_metrics,n_conditions);
                % Update the system dynamics
                obj.model.update(q, zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1));
                % For each metric compute the value of the point
                for j = 1:n_metrics
                    if(j<=n_metrics_prev)
                        if(~isempty(workspace_prev{i}))
                            % The metric is at valid value
                            wp.addMetric(workspace_prev{i}.metrics{j,1},workspace_prev{i}.metrics{j,2},j);
                        end
                    else
                        % New metric
                        [metric_type,metric_value]          =   obj.metrics{j}.evaluate(obj.model,obj.options.solver_options);
                        % The metric is at valid value
                        wp.addMetric(metric_type,metric_value,j);
                    end
                end
                % Evaluate each condition
                for j = 1:length(obj.conditions)
                    if(j<=n_conditions_prev)
                        if((~isempty(workspace_prev{i}))&&(~isempty(workspace_prev{i}.conditions{j})))
                            % The metric is at valid value
                            wp.addCondition(workspace_prev{i}.conditions{j,1},j);
                        end
                    else
                        % New condition
                        [condition_type,condition_value]    =   obj.conditions{j}.evaluate(obj.model,wp);
                        if(condition_value==true)
                            % The condition is evaluated to be true
                            wp.addCondition(condition_type,j);
                        end
                    end
                end
                test_conditions = cellfun(@isempty,wp.conditions);
                % Determine whether to add to workspace
                if(obj.options.union)
                    entry_condition = (~isempty(obj.metrics)||(sum(test_conditions(:,1))~=n_conditions));
                else
                    entry_condition = (sum(test_conditions(:,1))==0);
                end
                if(entry_condition)
                    % Add the workspace point to the 
                    obj.workspace{i} = wp;
                    workspace_count = workspace_count + 1;
                end
            end
            obj.comp_time.point_construction = toc(point_t_in);
            graph_t_in = tic;
            % If graph generation is set generate the graph
            if(nargin == 4)
%                 obj.generateWorkspaceGraph(w_connectivity);
                obj.createWorkspaceGraph(w_conditions{1},[],w_connectivity);
                % FOR THE MOMENT NO METRICS
            end
            obj.comp_time.graph_construction = toc(graph_t_in);
            obj.comp_time.total = toc(total_t_in);
        end
        
        % Plotting function to plot a two dimensional (subset of the) workspace plot
        function plotWorkspace2(obj,plot_axis,capability_measure,slices,fixed_dim_cor)
            CASPR_log.Assert((isempty(slices)||(numel(slices)==2)),'Only 2 dimensional slices can be plotted in this function');
			CASPR_log.Assert((obj.model.numDofs-numel(fixed_dim_cor))==2,'The number of elements of fixed dimension are not equal to remaining dimension.');
            if(isempty(plot_axis))
                f = figure(); plot_axis = axes;
            end
            hold(plot_axis,'on');
            if(isa(capability_measure,'WorkspaceMetricType'))
                plot_workspace_metric_2(obj,plot_axis,capability_measure,slices);
            elseif(isa(capability_measure,'WorkspaceConditionType'))
                plot_workspace_condition_2(obj,plot_axis,capability_measure,slices,fixed_dim_cor,[],[]);
            else
                CASPR_log.Print('The capability measure must either be a workspace metric or a workspace condition',CASPRLogLevel.ERROR);
            end
            hold(plot_axis,'off');
        end
        
        % Plotting function to plot a two dimensional (subset of the) workspace plot with slider
        function plotWorkspaceSlide2(obj,plot_axis,capability_measure,slices,fixed_dim_cor,slide_dim_index)
            CASPR_log.Assert((isempty(slices)||(numel(slices)==2)),'Only 2 dimensional slices can be plotted in this function');
            CASPR_log.Assert((obj.model.numDofs-numel(fixed_dim_cor)-numel(slide_dim_index))==2,'The number of fixed dimension are not equal to remaining dimension.');
            CASPR_log.Assert(numel(slide_dim_index)==1,'Only one axis can be slided.');
            if(isempty(plot_axis))
                f = figure; plot_axis = axes;
            end
             hold(plot_axis,'on');
            if(isa(capability_measure,'WorkspaceMetricType'))
                plot_workspace_metric_2(obj,plot_axis,capability_measure,slices);
            elseif(isa(capability_measure,'WorkspaceConditionType'))
                % create a slider with bounds [-1,1]
                b = uicontrol('Parent',f,'Style','slider','Position',[0,0,400,20],'value',0.5, 'min',0, 'max',1);
                b.Callback = @(es,ed) refreshdata(f,plot_workspace_condition_2(obj,plot_axis,capability_measure,slices,fixed_dim_cor,slide_dim_index,...
                floor(es.Value*(obj.grid.q_length(slide_dim_index)-1))*obj.grid.delta_q(slide_dim_index)+obj.grid.q_begin(slide_dim_index)));

            else
                CASPR_log.Print('The capability measure must either be a workspace metric or a workspace condition',CASPRLogLevel.ERROR);
            end
             hold(plot_axis,'off');
        end
        
        % Plotting function to plot a three dimensional (subset of the) workspace plot
        function plotWorkspace3(obj,plot_axis,capability_measure,slices,fixed_dim_cor)
            CASPR_log.Assert((isempty(slices)||(numel(slices)==3)),'Only 3 dimensional slices can be plotted in this function');
			CASPR_log.Assert((obj.model.numDofs-numel(fixed_dim_cor))==3,'The number of elements of fixed dimension are not equal to remaining dimension.');
            if(isempty(plot_axis))
                f = figure; plot_axis = axes; 
            end
            hold(plot_axis,'on');
            if(isa(capability_measure,'WorkspaceMetricType'))
                plot_workspace_metric_3(obj,plot_axis,capability_measure,slices);
            elseif(isa(capability_measure,'WorkspaceConditionType'))
                plot_workspace_condition_3(obj,plot_axis,capability_measure,slices,fixed_dim_cor,[],[]);
            else
                CASPR_log.Print('The capability measure must either be a workspace metric or a workspace condition',CASPRLogLevel.ERROR);
            end
            hold(plot_axis,'off');
        end
    
        % Plotting function to plot a three dimensional (subset of the) workspace plot with slider
        function plotWorkspaceSlide3(obj,plot_axis,capability_measure,slices,fixed_dim_cor,slide_dim_index)
            CASPR_log.Assert((isempty(slices)||(numel(slices)==3)),'Only 3 dimensional slices can be plotted in this function');
			CASPR_log.Assert((obj.model.numDofs-numel(fixed_dim_cor)-numel(slide_dim_index))==3,'The number of elements of fixed dimension are not equal to remaining dimension.');
            CASPR_log.Assert(numel(slide_dim_index)==1,'Only one axis can be slided.');

            if(isempty(plot_axis))
                f = figure; plot_axis = axes; 
            end
            hold(plot_axis,'on');
            if(isa(capability_measure,'WorkspaceMetricType'))
                plot_workspace_metric_3(obj,plot_axis,capability_measure,slices);
            elseif(isa(capability_measure,'WorkspaceConditionType'))
                b = uicontrol('Parent',f,'Style','slider','Position',[0,0,400,20],'value',0.5, 'min',0, 'max',1);
                b.Callback = @(es,ed) refreshdata(f,plot_workspace_condition_3(obj,plot_axis,capability_measure,slices,fixed_dim_cor,slide_dim_index,...
                floor(es.Value*(obj.grid.q_length(slide_dim_index)-1))*obj.grid.delta_q(slide_dim_index)+obj.grid.q_begin(slide_dim_index)));
            else
                CASPR_log.Print('The capability measure must either be a workspace metric or a workspace condition',CASPRLogLevel.ERROR);
            end
%             hold(plot_axis,'off');
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
            number_edges = size(obj.graph_rep,1);
            for i = 1:obj.grid.n_dimensions+metric_flag
                figure;
                edge_colour_matrix = nan(number_edges,1);
                for ii = 1:number_edges
                    if(obj.graph_rep(ii,2+i))
                        edge_colour_matrix(ii) = obj.workspace{obj.node_list(obj.graph_rep(ii,1),1)}.pose(i);
                    end
                end
                ax1 = axes;
                p = plot(G,'MarkerSize',2);
                layout(p,'auto')  
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
                    p.NodeCData = G.Nodes.NodeColours;
                else
                    p.NodeCData = (zero_colour-0.1)*ones(G.numnodes,1);
                end
                if(i <= obj.grid.n_dimensions)
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
        
%         function plotWorkspaceGraph(obj,plot_axis,~,~)
%             CASPR_log.Assert(~isempty(obj.edge_matrix),'Cannot plot without a valid graph object');
%             if(nargin == 1 || isempty(plot_axis))
%                 figure; plot_axis = axes; 
%             end
%             imagesc(plot_axis,obj.edge_matrix);            
%         end
        
        % Converts the workspace structure into an array
        function wsim_matrix = toMatrix(obj,capability_measure)
            % Matrix variable is either a metric or a condition. Determine
            % which one.
            if(isa(capability_measure,'WorkspaceMetricType'))
                point_flag = 1;
                % Find the index associated with the metric
            else
                point_flag = 0;
            end
            c_i = obj.find_capability_index(capability_measure);
            n = obj.grid.q_length;
            wsim_matrix = zeros(n);
            % Construct the mapping vector
            mapping_vector = zeros(obj.grid.n_dimensions,1); mapping_vector(1) = 1;
            for i=2:obj.grid.n_dimensions
                mapping_vector(i) = prod(obj.grid.q_length(1:i-1));
            end
            
            for i=1:length(obj.workspace)
                if(~isempty(obj.workspace{i}))
                    % Set the correct list
                    if(point_flag)
                        point_list = obj.workspace{i}.metrics;
                    else
                        point_list = obj.workspace{i}.conditions;
                    end
                    if(~isempty(point_list{c_i,1}))
                        index_vec = (obj.workspace{i}.pose - obj.grid.q_begin)./obj.grid.delta_q + [1;zeros(obj.grid.n_dimensions-1,1)];
                        for j = 1:obj.grid.n_dimensions
                            if(isnan(index_vec(j)))
                                index_vec(j) = 0;
                            end
                        end
                        index_num = int32(index_vec.'*mapping_vector);
                        wsim_matrix(index_num) = point_list{c_i,2};
                    end
                end
            end
        end
        
        % Converts a matrix back into the workspace object
        function toWorkspace(obj,wsim_matrix,capability)
            CASPR_log.Assert(isa(capability,'WorkspaceMetricType')||isa(capability,'WorkspaceConditionType'),'Capability must be a workspace metric or condition');
            if(isa(capability,'WorkspaceMetricType'))
                point_flag = 1; sentinel = 1i;
            else
                point_flag = 0; sentinel = 0;
            end
            k = 1; n_d = obj.grid.n_dimensions; index_vec = zeros(n_d,1);
            for i=1:numel(A)
                if(wsim_matrix(i)~=sentinel)
                    % Translate i into its coordinates
                    i_comp = i;
                    for j=n_d-1:-1:1
                        factor = prod(obj.q_length(1:j));
                        index_vec(j+1) = idivide(int32(i_comp),int32(factor));
                        i_comp = mod(i_comp,factor);
                    end
                    index_vec(1) = i_comp;
                    obj.workspace{k} = PointWorkspaceElement(obj.grid.q_begin + index_vec*obj.grid.delta_q,point_flag,~point_flag);
                    if(isa(capability,'WorkspaceMetricType'))
                        obj.workspace{k}.addMetric({capability,wsim_matrix(i)})
                    else
                        obj.workspace{k}.addCondition({capability,wsim_matrix(i)})
                    end
                    k = k+1;
                end
            end
        end
        
%         function generateWorkspaceGraph(obj,w_connectivity)
%             % Determine the number of nodes
%             number_nodes = length(obj.workspace) - sum(cellfun(@isempty,obj.workspace));
%             % Initialise a matrix based graph representation (THIS CAN BE
%             % CHANGED IF DESIRED)
%             obj.edge_matrix = eye(number_nodes); % Node a node is always connected to itself
%             i_counter = 1;
%             for i = 1:number_nodes
%                 % Skip over the empty cell entries
%                 while(isempty(obj.workspace{i_counter}))
%                     i_counter = i_counter + 1;
%                 end
%                 j_counter = i_counter+1;
%                 for j= i+1:number_nodes
%                     % Skip over the empty cell entries
%                     while(isempty(obj.workspace{j_counter}))
%                         j_counter = j_counter + 1;
%                     end
%                     % Evaluate the connectivity between the two objects
%                     [~,obj.edge_matrix(i,j),~] = w_connectivity.evaluate(obj.model,obj.workspace{i_counter},obj.workspace{j_counter});
%                     obj.edge_matrix(j,i) = obj.edge_matrix(i,j); % Due to bi-directional edges being assumed for the moment
%                     j_counter = j_counter+1;
%                 end
%                 i_counter = i_counter + 1;
%             end
%         end
        
        
        % Filters the workspace metric to set new limits.
        function filterWorkspaceMetric(obj,w_metric,metric_min,metric_max)
            m_i = find_capability_index(obj,w_metric);
            CASPR_log.Assert(m_i~=0,'Can only filter metrics that have already been computed');
            obj.metrics{m_i}.setMetricLimits(metric_min,metric_max);
            for i = 1:length(obj.workspace)
                wp = obj.workspace{i};
                if(wp.metrics{m_i,2} > metric_max)
                    wp.metrics{m_i,2} = metric_max;
                elseif(wp.metrics{m_i,2} < metric_min)
                    wp.metrics{m_i,2} = metric_min;
                end
            end
        end
            
    end
    
    methods (Access=private)
        function createWorkspaceGraph(obj,condition,metric,w_connectivity)
            if(isempty(metric))
                metric_flag = 0;
            else
                metric_flag = 1;
            end
            
            % For the moment this will be written as if there was only one
            % condition I will subsequently modify
            number_points = length(obj.workspace);
            % Create list of nodes
            obj.node_list = zeros(number_points,1+obj.grid.n_dimensions); 
            number_node = 0;
            % For each point
            for i = 1:number_points
                % Determine the number of the condition
                % Determine the number of the condition
                if(~isempty(obj.workspace{i}))
                    n_constraints = size(obj.workspace{i}.conditions,1);
                    for j = 1:n_constraints
                        if(condition.type == obj.workspace{i}.conditions{j,1})
                            number_node = number_node + 1;
                            obj.node_list(number_node,:) = [i,obj.workspace{i}.pose'];
                            break;
                        end
                    end
                end
            end
            % Resize to the correct size
            obj.node_list = obj.node_list(1:number_node,:); 
            
            % Computation for the maximum number of edges
            max_edges = number_node*max(obj.grid.q_length)*obj.grid.n_dimensions;
            
            % Initialise an adjacency list
            obj.graph_rep = zeros(max_edges,2+obj.grid.n_dimensions+metric_flag); % NEEDS TO BE CHANGED
            number_intersects = 0;
            for i = 1:number_node
                for j = i+1:number_node
                   % Check for connectivity
                   [~,is_connected,~] = w_connectivity.evaluate(obj.model,obj.workspace{obj.node_list(i,1)},obj.workspace{obj.node_list(j,1)});
                   if(is_connected)
                       number_intersects = number_intersects+1;
                       obj.graph_rep(number_intersects,1) = i;
                       obj.graph_rep(number_intersects,2) = j;
                       common_indices = obj.workspace{obj.node_list(i,1)}.pose == obj.workspace{obj.node_list(j,1)}.pose;
                       obj.graph_rep(number_intersects,3:2++obj.grid.n_dimensions) = common_indices';
                       % FOR THE MOMENT NO METRICS AND NO OTHER STUFF
                   end
                end
            end
            obj.graph_rep = obj.graph_rep(1:number_intersects,:);
        end
        
        % Finds the index at which a given condition or metric exists
        function c_i = find_capability_index(obj,capability)
            if(isa(capability,'WorkspaceMetricType'))
                capability_list = obj.metrics;
            else
                capability_list = obj.conditions;
            end
            c_i = 0;
            for i = 1:size(capability_list,2)
                if(capability == capability_list{i}.type)
                    c_i = i;
                    return;
                end
            end
        end
        
        % Plot the workspace for condition 2 objects
        function f = plot_workspace_condition_2(obj,plot_axis,w_condition,pose_index,fixed_dim_cor,slide_dim_index,slide_dim_cor)
            CASPR_log.Assert(~isempty(obj.conditions),'There are no conditions to plot');
            % Find the position of the condition in the workspace list
            c_i = obj.find_capability_index(w_condition);            
            CASPR_log.Assert(c_i~=0,'Workspace condition must be a workspace simulator condition');
            % Place the workspace information into vectors for plotting
            fixed_dim_index = 1:obj.model.numDofs;
            fixed_dim_index = setdiff(fixed_dim_index,pose_index);
            fixed_dim_index = setdiff(fixed_dim_index,slide_dim_index);
            plotting_workspace = obj.workspace;
            plot_x_in = []; plot_y_in = [];
            plot_x_out = []; plot_y_out = [];
            for i =1:size(plotting_workspace,1)
                % Search for suitable workspace point in specific coordinate
                if(~isempty(plotting_workspace{i}) && all(abs(plotting_workspace{i}.pose(fixed_dim_index)-fixed_dim_cor)<1e-6)...
                        && all(abs(plotting_workspace{i}.pose(slide_dim_index)-slide_dim_cor)<1e-6))
                    if(isempty(plotting_workspace{i}.conditions{c_i,1}))
                        q = obj.grid.getGridPoint(i);
                        plot_x_out = [plot_x_out,q(pose_index(1))]; %#ok<*AGROW>
                        plot_y_out = [plot_y_out,q(pose_index(2))];
                    else
                        % The point is in the workspace
                        plot_x_in = [plot_x_in,plotting_workspace{i}.pose(pose_index(1))];
                        plot_y_in = [plot_y_in,plotting_workspace{i}.pose(pose_index(2))];
                    end
                end
            end
            % If this doesn't work replace in_list with direct storage
            % Delete the previous figure data
            delete(findall(plot_axis,'type','Line'));

            % Plot the workspace points in black color
            f = plot(plot_axis,plot_x_in,plot_y_in,'Color',[0,0,0],'Marker','.','LineStyle','none');
            
            %Set the limit and annotation
            plot_axis.XLim = [obj.grid.q_begin(pose_index(1)) obj.grid.q_end(pose_index(1))];
            plot_axis.YLim = [obj.grid.q_begin(pose_index(2)) obj.grid.q_end(pose_index(2))];
            xlabel(sprintf('Axis %.0f', pose_index(1))); ylabel(sprintf('Axis %.0f', pose_index(2)));
            plot_axis.OuterPosition = [0.1 0.1 0.9 0.9];
            
            % Delete the previous annotaion
            delete(findall(gcf,'type','annotation'));
            % Print the axis information
            annotation_height = 0.9;
            for i = 1: numel(fixed_dim_index)
                annotation('textbox',[0 annotation_height .2 .05],'String',sprintf('Axis %.0f = %.2f',fixed_dim_index(i), fixed_dim_cor(i)),'FontSize',8,'LineStyle','none');
                annotation_height = annotation_height-0.05;
            end
            for i = 1: numel(slide_dim_index)
                annotation('textbox',[0 annotation_height .2 .05],'String',sprintf('Axis %.0f = %.2f',slide_dim_index(i), slide_dim_cor(i)),'FontSize',8,'LineStyle','none');
                annotation_height = annotation_height-0.05;
            end
%   
            % Plot other grid points in white color. This will block the actual workspace point, is suggested to be turned off.
            % plot(plot_axis,plot_x_out,plot_y_out,'Color',[1,1,1],'Marker','.','LineStyle','none')
        end
        
        % Plot the workspace for condition 2 objects
        function f = plot_workspace_condition_3(obj,plot_axis,w_condition,pose_index,fixed_dim_cor,slide_dim_index,slide_dim_cor)
            CASPR_log.Assert(~isempty(obj.conditions),'There are no conditions to plot')
            % Find the position of the condition in the workspace list
            c_i = obj.find_capability_index(w_condition);            
            CASPR_log.Assert(c_i~=0,'Workspace condition must be a workspace simulator condition');
            % Place the workspace information into vectors for plotting
            fixed_dim_index = 1:obj.model.numDofs;
            fixed_dim_index = setdiff(fixed_dim_index,pose_index);
            fixed_dim_index = setdiff(fixed_dim_index,slide_dim_index);
            plotting_workspace = obj.workspace;
            plot_x_in = []; plot_y_in = []; plot_z_in = [];
            plot_x_out = []; plot_y_out = []; plot_z_out = [];
            for i =1:size(plotting_workspace,1)
                if(~isempty(plotting_workspace{i}) && all(abs(plotting_workspace{i}.pose(fixed_dim_index)-fixed_dim_cor)<1e-10)...
                        && all(abs(plotting_workspace{i}.pose(slide_dim_index)-slide_dim_cor)<1e-10))
                    if(isempty(plotting_workspace{i}.conditions{c_i,1}))
                        q = obj.grid.getGridPoint(i);
                        plot_x_out = [plot_x_out,q(pose_index(1))];
                        plot_y_out = [plot_y_out,q(pose_index(2))];
                        plot_z_out = [plot_z_out,q(pose_index(3))];
                        
                    else
                        % The point is in the workspace
                        plot_x_in = [plot_x_in,plotting_workspace{i}.pose(pose_index(1))];
                        plot_y_in = [plot_y_in,plotting_workspace{i}.pose(pose_index(2))];
                        plot_z_in = [plot_z_in,plotting_workspace{i}.pose(pose_index(3))];
                    end
                end
            end
            % If this doesn't work replace in_list with direct storage
            % Delete the previous figure data
            delete(findall(plot_axis,'type','Line'));
            % Plot the workspace points in black color
            f = plot3(plot_axis,plot_x_in,plot_y_in,plot_z_in,'Color',[0,0,0],'Marker','.','LineStyle','none');
            
            %Set the limit and annotation
            plot_axis.XLim = [obj.grid.q_begin(pose_index(1)) obj.grid.q_end(pose_index(1))];
            plot_axis.YLim = [obj.grid.q_begin(pose_index(2)) obj.grid.q_end(pose_index(2))];
            plot_axis.ZLim = [obj.grid.q_begin(pose_index(3)) obj.grid.q_end(pose_index(3))];
            xlabel(sprintf('Axis %.0f', pose_index(1))); ylabel(sprintf('Axis %.0f', pose_index(2))); zlabel(sprintf('Axis %.0f', pose_index(3)));
            plot_axis.OuterPosition = [0.1 0.1 0.9 0.9];

            % Delete the previous annotaion
            delete(findall(gcf,'type','annotation'));
            % Print the axis information
            annotation_height = 0.9;
            for i = 1: numel(fixed_dim_index)
                annotation('textbox',[0 annotation_height .2 .05],'String',sprintf('Axis %.0f = %.2f',fixed_dim_index(i), fixed_dim_cor(i)),'FontSize',8,'LineStyle','none');
                annotation_height = annotation_height-0.05;
            end
            for i = 1: numel(slide_dim_index)
                annotation('textbox',[0 annotation_height .2 .05],'String',sprintf('Axis %.0f = %.2f',slide_dim_index(i), slide_dim_cor(i)),'FontSize',8,'LineStyle','none');
                annotation_height = annotation_height-0.05;
            end
            % Plot other grid points in white color. This will block the actual workspace point, is suggested to be turned off.
            % plot3(plot_axis,plot_x_out,plot_y_out,plot_z_out,'Color',[1,1,1],'Marker','.','LineStyle','none')
        end
        
        % Plot the workspace metric for 2 and 3 dimensional objects
        function plot_workspace_metric_2(obj,plot_axis,w_metric,pose_index)
            CASPR_log.Assert(~isempty(obj.metrics),'There are no metrics to plot');
            plotting_workspace = obj.workspace;
            % First determine the metric entry
            m_i = obj.find_capability_index(w_metric);
            CASPR_log.Assert((abs(obj.metrics{m_i}.metricMax)~=Inf)&&(abs(obj.metrics{m_i}.metricMin)~=Inf),'Cannot plot a metric with infinite limits, filter the workspace points');
            CASPR_log.Assert(m_i~=0,'Workspace metric must be a workspace simulator metric');
            % There are probably three options here
            % 1. Make the user give in a plotting range for the metric (undesired) 
            % 2. Assume that the metric min and max are finite. If there
            % are not give an error asking the user to change the value.
            % 3. Run two passes through of the data.
            % Go through the 
            w_array = [obj.workspace{:}];
            w_array_metrics = [w_array.metrics]; 
            metric_array = [w_array_metrics{m_i,2:2:length(w_array_metrics)}];
%             mw = obj.metrics{m_i}.metricMax - obj.metrics{m_i}.metricMin + 1;
            mw = max(metric_array) - obj.metrics{m_i}.metricMin;
            scale_factor = 255/mw;
            c_map = colormap(flipud(gray(floor(scale_factor*mw)+1)));
            for i =1:size(plotting_workspace,1)
                wp = plotting_workspace{i};
                if(~isempty(wp))
                    % Find which metric entry to use
                    if(wp.metrics{m_i,2} == obj.metrics{m_i}.metricMin)
                        plot(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),'r.')
%                     elseif(wp.metrics{m_i,2} == obj.metrics{m_i}.metricMin)
%                         plot(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),'r.')
                    else
                        c = c_map(floor(scale_factor*(wp.metrics{m_i,2} - obj.metrics{m_i}.metricMin))+1,:);
                        plot(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),'Color',c,'Marker','.')
                    end
                end
            end
            axis([obj.grid.q_begin(pose_index(1)),obj.grid.q_end(pose_index(1)),obj.grid.q_begin(pose_index(2)),obj.grid.q_end(pose_index(2))]);
        end
        
        % Plot the workspace metric for 2 and 3 dimensional objects
        function plot_workspace_metric_3(obj,plot_axis,w_metric,pose_index)
            CASPR_log.Assert(~isempty(obj.metrics),'There are no metrics to plot')
            plotting_workspace = obj.workspace;
            % First determine the metric entry
            % FIGURE OUT HOW TO PREALLOCATE
            for i = 1:length(obj.metrics)
                type_list(i) = obj.metrics{i}.type;
            end
            metric_index = find(type_list==w_metric);
            CASPR_log.Assert(~isempty(metric_index),'The desired metric has not been computed');
            mw = obj.metrics{metric_index}.metricMax - obj.metrics{metric_index}.metricMin + 1;
            scale_factor = 255/mw;
            c_map = colormap(flipud(gray(floor(scale_factor*mw))));
            for i =1:size(plotting_workspace,1)
                wp = plotting_workspace{i};
                for j=1:size(wp.metrics,1)
                    metric_list(j) = wp.metrics{j,1};
                end
                if(isempty(wp.metrics)||(sum(w_metric == metric_list)==0))
                    c = [1,1,1];
                else
                    % Find which metric entry to use
                    c = c_map(floor(scale_factor*(wp.metrics{metric_list==w_metric,2} - obj.metrics{metric_index}.metricMin))+1,:);
                end
                plot3(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),wp.pose(pose_index(3)),'Color',c,'Marker','.')
            end
        end
    end    
end
