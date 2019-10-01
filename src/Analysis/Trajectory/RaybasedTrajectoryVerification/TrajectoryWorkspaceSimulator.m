% The simulator to run a trajectories simulation analysis to find out the
% time range within the workspace
%
% Author        : Paul Cheng
% Created       : 2019
% Description    :
% Workspace simulator generates the workspace over a defined trajectory.

classdef TrajectoryWorkspaceSimulator < SimulatorBase
    
    properties
        workspace               % Final trajectory workspace (output)
    end
    
    properties (SetAccess = private)
        comp_time_total             % Total time to compute workspace
        trajectories = []           % sets of trajectories
        conditions = []         % A list of conditions to be evaluated for
        trajectory_types = []
        time_range = []
    end
    properties (Hidden)
        
    end
    
    methods
        % The constructor for the workspace simulator class.
        function t = TrajectoryWorkspaceSimulator(model,conditions,trajectory_types,trajectories,time_range)
            t@SimulatorBase(model);
            t.conditions    = conditions;
            t.trajectory_types    = trajectory_types;
            t.trajectories  = trajectories;
            t.time_range = time_range;
        end
        
        % Implementation of the run function
        function run(obj)
            
            obj.workspace = TrajectoryWorkspace(obj.model);
            count_time = 0;c_2 = tic;tic
            total_t_in = tic;
            k = 1;
            workspace_count = 1;
            for i = 1:size(obj.trajectories,2) 
%                 i
                wr = TrajectoryWorkspaceElement(obj.model,obj.conditions,obj.trajectory_types,obj.trajectories{i},obj.time_range);
                
                entry_condition = 1;
                if(entry_condition)
                    % Add the workspace 
                    obj.workspace.trajectories{k} = wr;
                    workspace_count = workspace_count + 1;
                end
                k = k+1;
                count_time = count_time + toc;
                if toc(c_2) >= 2
                    CASPR_log.Print([sprintf('Working on %d ',i),sprintf('trajectory. Completion Percentage: %3.2f.',100*i/size(obj.trajectories,2)),sprintf(' Estimate %3.2f sec left',(count_time *size(obj.trajectories,2)/i - count_time))],CASPRLogLevel.INFO);  
                    c_2 = tic;
                end
                
                
            end
            obj.comp_time_total = toc(total_t_in);
            obj.workspace.trajectories = obj.workspace.trajectories';
        end
    end
end
