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
        conditions      % A list of conditions to be evaluated for
        metrics         % A list of metrics to be evaluated for
        options         % The workspace simulator options
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = WorkspaceSimulator(model,grid,w_conditions,w_metrics,options)
            w@Simulator(model);
            w.grid          = grid;
            w.conditions    = w_conditions;
            w.metrics       = w_metrics;
            w.options       = options;
        end
        
        % Implementation of the run function
        function run(obj, workspace_in)
            if(isempty(workspace_in))
                % Create a cell array for workspace
                obj.workspace = cell(obj.grid.n_points,1);
            else
                if(obj.options.full_storage)
                    % All grid points are stored in the workspace
                    % Confirm that the input workspace is valid
                    assert((length(workspace_in)==obj.grid.n_points),'The input workspace is invalid for full storage mode');
                end
            end
            workspace_count = 0;
            % Runs over the grid and evaluates the workspace condition at
            % each point
            for i = 1:obj.grid.n_points
                disp(i)
                % Get the grid point
                q = obj.grid.getGridPoint(i);
                % Construct the workspace point
                wp = WorkspacePoint(q);
                % Update the system dynamics
                obj.model.update(q, zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1));
                % For each metric compute the value of the point
                for j = 1:length(obj.metrics)
                    metric_cell    =   obj.metrics{j}.evaluate(obj.model,[]);
                    % MAKE A DECISION REGARDING WHAT TO DO WITH THE METRICS
                    wp.addMetric(metric_cell);
                end
                % Evaluate each condition
                for j = 1:length(obj.conditions)
                    condition_cell    =   obj.conditions{j}.evaluate(obj.model,wp);
                    if(obj.options.full_storage||(condition_cell{2}==true))
                        wp.addCondition(condition_cell);
                    end
                end
                if(~isempty(wp.metrics)||~isempty(wp.conditions))
                    % Add the workspace point to the 
                    obj.workspace{workspace_count+1} = wp;
                    workspace_count = workspace_count + 1;
                end
            end
            obj.workspace = obj.workspace(1:workspace_count);
        end
        
        % Plotting function to plot workspace conditions and/or metrics.
        % This is limited to 2 or 3 dimensions
        function plotWorkspace(obj,capability_measure,slices,plot_axis)
            assert(numel(slices)<=3,'Only 2 or 3 slices can be plotted at a time');
            if(isempty(plot_axis))
                figure; plot_axis = axes; 
            end
            hold(plot_axis,'on');
            if(isa(capability_measure,'WorkspaceMetricType'))
                plotWorkspaceMetric(obj,capability_measure,plot_axis,slices);
            elseif(isa(capability_measure,'WorkspaceConditionType'))
                plotWorkspaceCondition(obj,capability_measure,plot_axis,slices);
            else
                error('The capability measure must either be a workspace metric or a workspace condition');
            end
            hold(plot_axis,'off');
        end
        
        % Converts the workspace structure into an array
        function wsim_matrix = toMatrix(obj,colour_src)
            % Matrix variable is either a metric or a condition. Determine
            % which one.
            if(isa(colour_src,'WorkspaceMetricType'))
                point_flag = 1;
            else
                point_flag = 0;
            end
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
                if(~isempty(point_list))
                    for j = 1:length(obj.metrics)
                        type_list(j) = point_list{j}{1};
                    end
                    val_index = find(type_list==colour_src);
                    if(~isempty(val_index))
                        index_vec = (obj.workspace{i}.pose - obj.grid.q_begin)./obj.grid.delta_q + [1;zeros(obj.grid.n_dimensions-1,1)];
                        index_num = int32(index_vec.'*mapping_vector);
                        wsim_matrix(index_num) = point_list{val_index}{2};
                    end
                end
            end
        end
        
        % Converts a matrix back into the workspace object
        function toWorkspace(obj,wsim_matrix,capability)
            assert(isa(capability,'WorkspaceMetricType')||isa(capability,'WorkspaceConditionType'),'Capability must be a workspace metric or condition')
            k = 1; n_d = obj.grid.n_dimensions; index_vec = zeros(n_d,1);
            for i=1:numel(A)
                % Has to be changed for centinel
                if(wsim_matrix(i)>0)
                    % Translate i into its coordinates
                    i_comp = i;
                    for j=n_d-1:-1:1
                        factor = prod(obj.q_length(1:j));
                        index_vec(j+1) = idivide(int32(i_comp),int32(factor));
                        i_comp = mod(i_comp,factor);
                    end
                    index_vec(1) = i_comp;
                    obj.workspace{k} = WorkspacePoint(obj.grid.q_begin + index_vec*obj.grid.delta_q);
                    if(isa(capability,'WorkspaceMetricType'))
                        obj.workspace{k}.addMetric({capability,wsim_matrix(i)})
                    else
                        obj.workspace{k}.addCondition({capability,wsim_matrix(i)})
                    end
                    k = k+1;
                end
            end
        end
    end
    
    methods (Access=private)
        % Plot the workspace for 2 and 3 dimensional objects
        function plotWorkspaceCondition(obj,w_condition,plot_axis,pose_index)
            assert(~isempty(obj.conditions),'There are no conditions to plot')
            plotting_workspace = obj.workspace;
            for i =1:size(plotting_workspace,1)
                wp = plotting_workspace{i};
                if(isempty(wp.conditions)||(sum(w_condition == wp.conditions{:}{1})==0))
                    c = [1,1,1];
                else
                    c = [0,0,0];
                end
                if(numel(pose_index)==2)
                    plot(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),'Color',c,'Marker','.')
                else
                    plot3(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),wp.pose(pose_index(3)),'Color',c,'Marker','.')
                end
            end
        end
        
        % Plot the workspace for 2 and 3 dimensional objects
        function plotWorkspaceMetric(obj,w_metric,plot_axis,pose_index)
            assert(~isempty(obj.metrics),'There are no metrics to plot')
            plotting_workspace = obj.workspace;
            % First determine the metric entry
            % FIGURE OUT HOW TO PREALLOCATE
            for i = 1:length(obj.metrics)
                type_list(i) = obj.metrics{i}.type;
            end
            metric_index = find(type_list==w_metric);
            assert(~isempty(metric_index),'The desired metric has not been computed');
            mw = obj.metrics{metric_index}.metricMax - obj.metrics{metric_index}.metricMin + 1;
            scale_factor = 255/mw;
            c_map = colormap(flipud(gray(floor(scale_factor*mw))));
            for i =1:size(plotting_workspace,1)
                wp = plotting_workspace{i};
                wp.metrics{:}{1}
                if(isempty(wp.metrics)||(sum(w_metric == wp.metrics{:}{1})==0))
                    c = [1,1,1];
                else
                    % Find which metric entry to use
                    c = c_map(floor(sf*(wp.metrics{wp.metrics{:}{1}==colour_src}{2} - obj.metrics{metric_index}.metricMin)));
                end
                if(numel(pose_index)==2)
                    plot(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),'Color',c,'Marker','.')
                else
                    plot3(plot_axis,wp.pose(pose_index(1)),wp.pose(pose_index(2)),wp.pose(pose_index(3)),'Color',c,'Marker','.')
                end
            end
        end
    end    
end

