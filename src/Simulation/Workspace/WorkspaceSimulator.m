% The simulator to run a workspace simulation
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
%   Workspace simulator generates the workspace over a defined set of space
%   (currently only a grid of states is accepted). The workspace simulation
%   performs essentially a numerical brute force approach over the set of
%   poses for a given workspace condition and/or metric.
classdef WorkspaceSimulator < Simulator
    
    properties
        grid            % Grid object for brute force workspace (input)
        workspace       % Final Workspace (output)
        conditions = [] % A list of conditions to be evaluated for
        metrics = []    % A list of metrics to be evaluated for
        options         % The workspace simulator options
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = WorkspaceSimulator(model,grid,options)
            w@Simulator(model);
            w.grid          = grid;
            w.options       = options;
        end
        
        % Implementation of the run function
        function run(obj, w_conditions, w_metrics)
            % First determine how big the metrics previously were
            if(isempty(obj.metrics))
                n_metrics_prev  = 0; 
                obj.metrics     = w_metrics;
            else
                n_metrics_prev  = size(obj.metrics,2);
                obj.metrics = [obj.metrics,w_metrics];
            end
            
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
                if((abs(w_metrics{i}.metricMax)==Inf)||(abs(w_metrics{i}.metricMin)==Inf))
                    CASPR_log.Print('A metric with infinite limit values cannot be plotted.  To plot please set the metric limit to be finite or filter the workspace after plotting',CASPRLogLevel.WARNING);
                end
            end
            
            % Create a cell array for workspace
            obj.workspace = cell(obj.grid.n_points,1);
            workspace_count = 0;
            n_metrics       = length(obj.metrics);
            n_conditions    = length(obj.conditions);
            % Determine translation from workspace_in to current metrics
            % list            
            
            % Runs over the grid and evaluates the workspace condition at
            % each point
            for i = 1:obj.grid.n_points
                CASPR_log.Print(sprintf('Workspace point %d',i),CASPRLogLevel.INFO);
                % Get the grid point
                q = obj.grid.getGridPoint(i);
                % Construct the workspace point
                wp = WorkspacePoint(q,n_metrics,n_conditions);
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
                        [metric_type,metric_value]          =   obj.metrics{j}.evaluate(obj.model,[]);
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
        end
        
        % Plotting function to plot a two dimensional (subset of the) workspace plot
        function plotWorkspace2(obj,capability_measure,slices,plot_axis)
            CASPR_log.Assert(numel(slices)==2,'Only 2 dimensional slices can be plotted in this function');
            if(isempty(plot_axis))
                figure; plot_axis = axes; 
            end
            hold(plot_axis,'on');
            if(isa(capability_measure,'WorkspaceMetricType'))
                plotWorkspaceMetric2(obj,capability_measure,plot_axis,slices);
            elseif(isa(capability_measure,'WorkspaceConditionType'))
                plotWorkspaceCondition2(obj,capability_measure,plot_axis,slices);
            else
                CASPR_log.Print('The capability measure must either be a workspace metric or a workspace condition',CASPRLogLevel.ERROR);
            end
            hold(plot_axis,'off');
        end
        
        % Plotting function to plot a three dimensional (subset of the)
        % workspace plot
        function plotWorkspace3(obj,capability_measure,slices,plot_axis)
            CASPR_log.Assert(numel(slices)==3,'Only 3 dimensional slices can be plotted in this function');
            if(isempty(plot_axis))
                figure; plot_axis = axes; 
            end
            hold(plot_axis,'on');
            if(isa(capability_measure,'WorkspaceMetricType'))
                plotWorkspaceMetric3(obj,capability_measure,plot_axis,slices);
            elseif(isa(capability_measure,'WorkspaceConditionType'))
                plotWorkspaceCondition3(obj,capability_measure,plot_axis,slices);
            else
                CASPR_log.Print('The capability measure must either be a workspace metric or a workspace condition',CASPRLogLevel.ERROR);
            end
            hold(plot_axis,'off');
        end
        
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
                % Set the correct list
                if(point_flag)
                    point_list = obj.workspace{i}.metrics;
                else
                    point_list = obj.workspace{i}.conditions;
                end
                if(~isempty(point_list{c_i,1}))
                    index_vec = (obj.workspace{i}.pose - obj.grid.q_begin)./obj.grid.delta_q + [1;zeros(obj.grid.n_dimensions-1,1)];
                    index_num = int32(index_vec.'*mapping_vector);
                    wsim_matrix(index_num) = point_list{c_i,2};
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
                    obj.workspace{k} = WorkspacePoint(obj.grid.q_begin + index_vec*obj.grid.delta_q,point_flag,~point_flag);
                    if(isa(capability,'WorkspaceMetricType'))
                        obj.workspace{k}.addMetric({capability,wsim_matrix(i)})
                    else
                        obj.workspace{k}.addCondition({capability,wsim_matrix(i)})
                    end
                    k = k+1;
                end
            end
        end
        
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
        function plotWorkspaceCondition2(obj,w_condition,plot_axis,pose_index)
            CASPR_log.Assert(~isempty(obj.conditions),'There are no conditions to plot');
            % Find the position of the condition in the workspace list
            c_i = obj.find_capability_index(w_condition);            
            CASPR_log.Assert(c_i~=0,'Workspace condition must be a workspace simulator condition');
            % Place the workspace information into vectors for plotting
            plotting_workspace = obj.workspace;
            plot_x_in = []; plot_y_in = [];
            plot_x_out = []; plot_y_out = [];
            for i =1:size(plotting_workspace,1)
                if(isempty(plotting_workspace{i})||isempty(plotting_workspace{i}.conditions{c_i,1}))
                    q = obj.grid.getGridPoint(i);
                    plot_x_out = [plot_x_out,q(pose_index(1))];
                    plot_y_out = [plot_y_out,q(pose_index(2))];
                else
                    % The point is in the workspace
                    plot_x_in = [plot_x_in,plotting_workspace{i}.pose(pose_index(1))];
                    plot_y_in = [plot_y_in,plotting_workspace{i}.pose(pose_index(2))];
                end
            end
            % If this doesn't work replace in_list with direct storage
            plot(plot_axis,plot_x_in,plot_y_in,'Color',[0,0,0],'Marker','.','LineStyle','none')
            plot(plot_axis,plot_x_out,plot_y_out,'Color',[1,1,1],'Marker','.','LineStyle','none')
        end
        
        % Plot the workspace for condition 2 objects
        function plotWorkspaceCondition3(obj,w_condition,plot_axis,pose_index)
            CASPR_log.Assert(~isempty(obj.conditions),'There are no conditions to plot')
            % Find the position of the condition in the workspace list
            c_i = obj.find_capability_index(w_condition);            
            CASPR_log.Assert(c_i~=0,'Workspace condition must be a workspace simulator condition');
            % Place the workspace information into vectors for plotting
            plotting_workspace = obj.workspace;
            plot_x_in = []; plot_y_in = []; plot_z_in = [];
            plot_x_out = []; plot_y_out = []; plot_z_out = [];
            for i =1:size(plotting_workspace,1)
                if(~isempty(plotting_workspace{i}.conditions{c_i,1}))
                    % The point is in the workspace
                    plot_x_in = [plot_x_in,plotting_workspace{i}.pose(pose_index(1))];
                    plot_y_in = [plot_y_in,plotting_workspace{i}.pose(pose_index(2))];
                    plot_z_in = [plot_z_in,plotting_workspace{i}.pose(pose_index(3))];
                else
                    q = obj.grid.getGridPoint(i);
                    plot_x_out = [plot_x_out,q(pose_index(1))];
                    plot_y_out = [plot_y_out,q(pose_index(2))];
                    plot_z_out = [plot_y_out,q(pose_index(3))];
                end
            end
            % If this doesn't work replace in_list with direct storage
            plot3(plot_axis,plot_x_in,plot_y_in,plot_z_in,'Color',[0,0,0],'Marker','.')
            plot3(plot_axis,plot_x_out,plot_y_out,plot_z_out,'Color',[1,1,1],'Marker','.')
        end
        
        % Plot the workspace metric for 2 and 3 dimensional objects
        function plotWorkspaceMetric2(obj,w_metric,plot_axis,pose_index)
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
            mw = obj.metrics{m_i}.metricMax - obj.metrics{m_i}.metricMin + 1;
            scale_factor = 255/mw;
            c_map = colormap(flipud(gray(floor(scale_factor*mw))));
            for i =1:size(plotting_workspace,1)
                wp = plotting_workspace{i};
                % Find which metric entry to use
                c = c_map(floor(scale_factor*(wp.metrics{m_i,2} - obj.metrics{m_i}.metricMin))+1,:);
                plot(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),'Color',c,'Marker','.')
            end
        end
        
        % Plot the workspace metric for 2 and 3 dimensional objects
        function plotWorkspaceMetric3(obj,w_metric,plot_axis,pose_index)
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

