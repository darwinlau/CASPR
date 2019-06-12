% Class to compute whether a two workspace points are neighbours on a uniform grid
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    : 
classdef UniformGridNeighbour < WorkspaceConnectivityBase
    properties (SetAccess = protected, GetAccess = protected)
        grid           % The grid for computation
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = UniformGridNeighbour(grid)
            w.grid = grid;
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function isConnected = evaluateFunction(obj,~,workspace_point_1,workspace_point_2)
            % FILE ME IN
            abs_diff_pose = abs(workspace_point_1.pose - workspace_point_2.pose);
            n_dim         = length(abs_diff_pose);
            if(sum(abs_diff_pose <=obj.grid.delta_q)==n_dim)
                % Connection without wrapping around in the configurations
                isConnected = 1;
            elseif(sum((abs_diff_pose <=obj.grid.delta_q) |((abs(abs_diff_pose -(obj.grid.q_end - obj.grid.q_begin))<=1e-6)&obj.grid.q_wrap))==n_dim)
                % Connection with wrapping around in the configurations
                isConnected = 1;
            else
                % No connection
                isConnected = 0;
            end 
        end
    end
end

