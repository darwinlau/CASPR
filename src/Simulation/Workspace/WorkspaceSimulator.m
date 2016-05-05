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
        WCondition      % The workspace condition
        metric          % The metric
        m_flag          % Flag to indicate whether metric can be used to look at workspace
    end
    
    methods        
        function id = WorkspaceSimulator(model,w_condition,metric)
            id@Simulator(model);
            id.WCondition   =   w_condition;
            if(nargin > 2)
                id.metric       =   metric;
            else
                id.metric       =   NullMetric();
            end
        end
        
        function run(obj, grid)
            obj.grid        =   grid;
            % Dimension is the dimension of each state in the grid + 1 for
            % the metric value
%             obj.grid.n_points
            obj.workspace   =   repmat([grid.getGridPoint(1);0],1,obj.grid.n_points);
            workspace_count =   0;
            % Runs over the grid and evaluates the workspace condition at
            % each point
            for i = 1:obj.grid.n_points
                disp(i)
                q = obj.grid.getGridPoint(i);
                obj.model.update(q, zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1));
                inWorkspace     =   obj.WCondition.evaluate(obj.model);
                if(~isa(obj.metric,'NullMetric'))
                    if(inWorkspace)
                        metricValue    =   obj.metric.evaluate(obj.model,[],obj.WCondition.method,inWorkspace);
                        workspace_count = workspace_count + 1;
                        obj.workspace(:,workspace_count) = [q;metricValue];
                    end
                else
                    if(inWorkspace)
                        workspace_count = workspace_count + 1;
                        obj.workspace(:,workspace_count) = [q;1];
                    end
                end
            end
            obj.workspace = obj.workspace(:,1:workspace_count);
        end
        
        function plotWorkspace(obj,workspace,plot_axis)
            n_d = obj.grid.n_dimensions;
%             assert(n_d<=3,'Dimension of Workspace too large to plot');
            if(isempty(workspace))
                plotting_workspace = obj.workspace;
            else
                plotting_workspace = workspace;
            end
%             figure;
            if(isempty(plot_axis))
                figure; plot_axis = axes; 
            end
            hold(plot_axis,'on');
            mw = ceil(max(plotting_workspace(n_d+1,:).*(plotting_workspace(n_d+1,:)~=Inf))+10);
            if(isempty(mw)||isnan(mw))
                mw = 1;
            end
            sf = 255/mw;
            map = colormap(flipud(gray(floor(sf*mw)))); % Look if there is a better colour map
            for i =1:size(plotting_workspace,2)
                if((n_d == 2))
                    axis([-180 180 -180 180]);
                    if(plotting_workspace(3,i)==Inf)
                        plot(plot_axis,(180/pi)*plotting_workspace(1,i),(180/pi)*plotting_workspace(2,i),'Color',map(sf*mw,:),'Marker','.')
                    else
%                         plot(plot_axis,plotting_workspace(1,i),plotting_workspace(2,i),'k.')
                        plot(plot_axis,(180/pi)*plotting_workspace(1,i),(180/pi)*plotting_workspace(2,i),'Color',map(int32(sf*(plotting_workspace(3,i)+10)),:),'Marker','.')
                    end
                elseif((n_d == 3)||(n_d == 6))
%                     axis([-180 180 -180 180 -180 180]);
                    axis([-4 4 -3 3 1 5])
                    if(plotting_workspace(4,i)==Inf)
                        plot3(plot_axis,(180/pi)*plotting_workspace(1,i),(180/pi)*plotting_workspace(2,i),(180/pi)*plotting_workspace(3,i),'Color',map(mw,:),'Marker','.')
                    else
%                         plot3(plot_axis,(180/pi)*plotting_workspace(1,i),(180/pi)*plotting_workspace(2,i),(180/pi)*plotting_workspace(3,i),'Color',map(int32(sf*(plotting_workspace(4,i)+10)),:),'Marker','.')
                        plot3(plot_axis,plotting_workspace(1,i),plotting_workspace(2,i),plotting_workspace(3,i),'k.')
                    end
                end
            end 
        end
        
        function plotWorkspaceHigherDimension(obj)
            % This function will always only plot the first two dimensions
            n_d = obj.grid.n_dimensions;
            figure;  hold on;    
            mw = ceil(max(obj.workspace(n_d+1,:).*(obj.workspace(n_d+1,:)~=Inf))+10);
            if(isnan(mw))
                mw = 1;
            end
            sf = 255/mw;
            map = colormap(flipud(gray(floor(sf*mw)))); % Look if there is a better colour map
            for i =1:size(obj.workspace,2)
                %                 axis([-180 180 -180 180]);
                if(obj.workspace(n_d+1,i)==Inf)
                    plot(obj.workspace(1,i),obj.workspace(2,i),'Color',map(sf*mw,:),'Marker','.')
                else
                    plot(obj.workspace(1,i),obj.workspace(2,i),'Color',map(int32(sf*(obj.workspace(n_d+1,i)+10)),:),'Marker','.')
                end
            end 
        end
        
        function plotWorkspaceInterior(obj)
        	assert(obj.grid.n_dimensions<=3,'Dimension of Workspace too large to plot');
            filtered_workspace = obj.boundary_filter;
            obj.plotWorkspace(filtered_workspace);
        end
        
        function plotWorkspaceComponents(obj,components)
            assert(obj.grid.n_dimensions<=3,'Dimension of Workspace too large to plot');
            [adjacency_matrix,laplacian_matrix] = obj.toAdjacencyMatrix();
            components = obj.findConnectedComponents(adjacency_matrix);
            figure;  hold on;   
            map = colormap(hsv(max(max(components))));
            for i =1:size(obj.workspace,2)
                if(obj.grid.n_dimensions == 2)
                    axis([-180 180 -180 180]);
                    plot((180/pi)*obj.workspace(1,i),(180/pi)*obj.workspace(2,i),'Color',map(components(i),:),'Marker','.')
                elseif(obj.grid.n_dimensions == 3)
                    axis([-180 180 -180 180 -180 180]);
                    plot3((180/pi)*obj.workspace(1,i),(180/pi)*obj.workspace(2,i),(180/pi)*obj.workspace(3,i),'Color',map(components(i),:),'Marker','.')
                end
            end 
        end
        
        function wsim_matrix = toMatrix(obj)
            if(obj.grid.n_dimensions == 2)
                n_x = int32(obj.grid.q_length(1));
                n_y = int32(obj.grid.q_length(2));
                wsim_matrix = zeros(n_x,n_y);
                for i=1:length(obj.workspace)
                    j = int32((obj.workspace(1,i) - obj.grid.q_begin(1))/obj.grid.delta_q(1) + 1);
                    k = int32((obj.workspace(2,i) - obj.grid.q_begin(2))/obj.grid.delta_q(2) + 1);
                    wsim_matrix(j,k) = obj.workspace(3,i);
                end
            elseif(obj.grid.n_dimensions == 3)
                n_x = obj.grid.q_length(1);
                n_y = obj.grid.q_length(2);
                n_z = obj.grid.q_length(3);
                wsim_matrix = zeros(n_x,n_y,n_z);
                for i=1:length(obj.workspace)
                    j = int32((obj.workspace(1,i) - obj.grid.q_begin(1))/obj.grid.delta_q(1) + 1);
                    k = int32((obj.workspace(2,i) - obj.grid.q_begin(2))/obj.grid.delta_q(2) + 1);
                    l = int32((obj.workspace(3,i) - obj.grid.q_begin(3))/obj.grid.delta_q(3) + 1);
                    wsim_matrix(j,k,l) = obj.workspace(4,i);
                end
            else
                disp('Dimension is too large');
            end
        end
        
        function toWorkspace(obj,wsim_matrix)
            k = 1;
            for i = 1:size(wsim_matrix,1)
                for j = 1:size(wsim_matrix,2)
                    if(wsim_matrix(i,j)>0)
                        obj.workspace(:,k) = [obj.grid.q_begin(1) + i*obj.grid.delta_q(1);obj.grid.q_begin(2) + j*obj.grid.delta_q(2);wsim_matrix(i,j)];
                        k = k+1;
                    end
                end
            end
            obj.workspace = obj.workspace(:,1:k);
        end
        
        
    end
    
    methods (Access=private)
        function boundaryFilter(obj)
            obj.filtered_workspace = zeros(size(obj.workspace));
            f_count = 1;
            n_nodes = size(obj.workspace,2);
            tol = 1e-6;
            for i = 1:n_nodes
                c_count = 0;
                j = 1;
                while((j<=n_nodes)&&(c_count<8))
                    % Check if points are connected
                    if((i~=j)&&(obj.WCondition.connected(obj.workspace,i,j,obj.grid)))
%                     if((obj.WCondition.connected(obj.workspace,i,j,obj.grid)))
                        c_count = c_count + 1;
                    end
                    
                    if(c_count == 8)
                        obj.filtered_workspace(:,f_count) = obj.workspace(:,i);
                        f_count = f_count+1;
                    end
                    j = j+1;
                end
                i
            end
            obj.filtered_workspace = obj.filtered_workspace(:,1:f_count-1);
        end
        
        function con_comp = findConnectedComponents(obj,adjacency_matrix)
            components = 0;
            
            con_comp = zeros(size(obj.workspace,2),1);
            
            % Write this better later
            disp('I am here')
            qh_count = 1;
            qe_count = 1;
            queue = con_comp;
            mark = queue;
            for i = 1:size(obj.workspace,2)
                if(con_comp(i) == 0)
                    components = components + 1;
                    mark(i) = 1;
                    queue(qh_count) = i;
                    qe_count = qe_count+1;
                    while((qh_count<qe_count)&&(queue(qh_count) ~= 0))
%                     while(qh_count < 4)
                        % pop vertex from the queue and assign to component
                        con_comp(queue(qh_count)) = components;
                        % add all adjacent elements to the queue that have
                        % not allready been added or popped
                        adj = find(adjacency_matrix(:,queue(qh_count)).*(mark==0)==1);
                        % adjust queue
                        if(size(adj,1)>0)
                            queue(qe_count:qe_count+size(adj,1)-1) = adj;
                            mark(adj) = 1;
                            qe_count = qe_count + size(adj,1);
                        end
                        qh_count = qh_count + 1;
                        
                    end
                else
                    
                end
            end 
        end
        
        function [adjacency_matrix,laplacian_matrix] = toAdjacencyMatrix(obj)
            n_nodes = size(obj.workspace,2);
            laplacian_matrix = zeros(n_nodes);
            adjacency_matrix = zeros(n_nodes);
            tol = 1e-6;
            for i = 1:n_nodes
                for j = i+1:n_nodes
                    % Check if points are connected
                    if(obj.WCondition.connected(obj.workspace,i,j,obj.grid))
                        adjacency_matrix(i,j) = 1;
                        adjacency_matrix(j,i) = 1;
                        laplacian_matrix(i,j) = -1;
                        laplacian_matrix(j,i) = -1;
                        laplacian_matrix(i,i) = laplacian_matrix(i,i) + 1;
                        laplacian_matrix(j,j) = laplacian_matrix(j,j) + 1;
                    end
                end
            end
        end
    end    
end

