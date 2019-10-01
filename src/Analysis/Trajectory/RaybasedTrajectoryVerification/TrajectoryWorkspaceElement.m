% A container class to hold workspace analysis information for a single
% trajectory with different condition(s)
%
% Author        : Paul Cheng
% Created       : 2019
% Description    : This class contains the information obtained
% through trajectory feasibility analysis.

classdef TrajectoryWorkspaceElement < handle
    properties(SetAccess = protected)   
        conditions = [];                    % Workspace conditions
        default_time_range = [];         % default trajectory time range
        feasible_time_range = []            % Valid time range path under condition(s)
        parametric_functions = [];          % Parametric functions of each dimension
        computation_time = 0;
        
    end
    
    properties 
        plotting = [];                      % Plot graph data
    end
    methods
        % Constructor for the class
        function wp = TrajectoryWorkspaceElement(model,conditions,trajectory_types,trajectory,time_range)
            wp.feasible_time_range = [];           
            wp.parametric_functions = [];           
            wp.plotting = [];
            wp.default_time_range = time_range; 
            wp.conditions = [];
            common_time_interval = wp.default_time_range; 
            common_time_interval_in_t = wp.default_time_range;
            
            for i = 1:size(conditions,2)
                condition_method(i) = conditions{i}.method;
            end
            % warning for different method may cause error
            if ~all(condition_method == condition_method(1))
                CASPR_log.Warn('Inaccurate result will be found due to the mix-usage of method, try to use the same analysis method')
            end
            % Unify the input
            [universal_trajectory,trajectory_time_range,maximum_trajectory_degree] = TrajectoryPreparation(model,trajectory_types,trajectory,wp.default_time_range);
            
            % evaluate for different conditions
            
            for j_c = 1:size(conditions,2)
                % keep variables: other_info,traj_interval for development usage
                [traj_interval{j_c},time_interval{j_c}, comp_time,other_info{j_c},theta] = conditions{j_c}.evaluate(model,universal_trajectory,trajectory_time_range,maximum_trajectory_degree);
                switch conditions{j_c}.method
                    case 'ANALYTICAL'
                        common_time_interval_in_t = IntersectedTimeRange(common_time_interval_in_t,time_interval{j_c});
                        common_time_interval_in_t = reshape(common_time_interval_in_t,[],2);   
                        time_for_translation = common_time_interval_in_t;
                        time_for_orientation = atan(common_time_interval_in_t*trajectory_time_range(2)*tan(theta/2))*2/theta;
%                         time_for_translation = common_time_interval_in_T/tan(theta/2);
%                         time_for_orientation = atan(common_time_interval_in_T)*2/theta;
                        common_time_interval = {time_for_translation,time_for_orientation};
                    otherwise
                     common_time_interval = IntersectedTimeRange(common_time_interval,time_interval{j_c});
                     common_time_interval = reshape(common_time_interval,[],2);   
                end
                wp.computation_time = comp_time + wp.computation_time;
            end
            if(~isempty(common_time_interval))
                wp.feasible_time_range = common_time_interval;
                wp.parametric_functions = universal_trajectory;
                wp.conditions =  conditions;
                
            else
                wp.feasible_time_range = [];
                wp.parametric_functions = universal_trajectory;
                wp.conditions =  conditions;
            end
        end
        
        
        
    end
end
% A function to unify different trajectories input to cfit type and return the trajectory
% time range. However it is set to be 0 to 1 for every type of trajectories
% since it is hard to decide what is the real time range
function [universal_trajectory,trajectory_time_range,maximum_trajectory_degree] = TrajectoryPreparation(model,trajectory_types,trajectory,time_range)
    start_times = [];end_times = []; time_lower_bound = []; time_upper_bound = [];
    default_time_range = time_range;
    q = trajectory;
    switch(trajectory_types)
        case 'PARAMETRIC'
            maximum_trajectory_degree = [];
            % Only consider the translation traj since orien traj may cause bugs
            q_upper_bound = [q(:,1:end-1),q(:,end) - model.bodyModel.q_max];
            q_lower_bound = [q(:,1:end-1),q(:,end) - model.bodyModel.q_min];
            for i = 1:size(q,1)
                if model.bodyModel.q_dofType(i) == 'TRANSLATION'
                    time_upper_bound = [time_upper_bound; real(roots(q_upper_bound(i,:)))];
                    time_lower_bound = [time_lower_bound; real(roots(q_lower_bound(i,:)))];
                end
            end
%             trajectory_time_range = [min(time_lower_bound),max(time_upper_bound)];
            trajectory_time_range = default_time_range;
            Message = ['Path time boundary set to: ',num2str(trajectory_time_range(1)),' to ', num2str(trajectory_time_range(2))];
            disp(Message)
            time_steps = linspace(trajectory_time_range(1),trajectory_time_range(2),1000);
            for i = 1:size(q,1)
                traj_data(i,:) = polyval(q(i,:),time_steps);
                universal_trajectory{i} = fit(time_steps',traj_data(i,:)','cubicinterp');
            end

        case 'POSE_WISE'
            maximum_trajectory_degree = [];
            trajectory_time_range = default_time_range;
            Message = ['Path time boundary set to: ',num2str(trajectory_time_range(1)),' to ', num2str(trajectory_time_range(2))];
            disp(Message)
            if ~ismember(model.numDofs,size(trajectory))
                CASPR_log.Error('Pose-wise trajectory format not correct. Should be q_i = [x,x,x]^T ');
            end
            time_steps = linspace(trajectory_time_range(1),trajectory_time_range(2),size(trajectory,2));
            
            for i = 1:size(q,1)
                universal_trajectory{i} = fit(time_steps',trajectory(i,:)','cubicinterp');
            end

        case 'TRIGONOMETRIC'
            maximum_trajectory_degree = [];
            trajectory_time_range = default_time_range;
            Message = ['Path time boundary set to: ',num2str(trajectory_time_range(1)),' to ', num2str(trajectory_time_range(2))];
            disp(Message)
            time_steps = linspace(trajectory_time_range(1),trajectory_time_range(2),1000);

            for i = 1:size(q,1)
                traj_data(i,:) = double(subs(trajectory{i},time_steps));
                universal_trajectory{i} = fit(time_steps',traj_data(i,:)','cubicinterp');
            end
        case 'CONTROL_POINTS'
            trajectory_time_range = default_time_range;
            Message = ['Path time boundary set to: ',num2str(trajectory_time_range(1)),' to ', num2str(trajectory_time_range(2))];
            disp(Message)
            maximum_trajectory_degree = 0;
            for i = 1:size(trajectory,1)
                if model.bodyModel.q_dofType(i) == 'TRANSLATION' 
                maximum_trajectory_degree = max(maximum_trajectory_degree,max(size(unique(trajectory{i})))-1);
                end
                time_steps = linspace(trajectory_time_range(1),trajectory_time_range(2),size(trajectory{i},2));
                universal_trajectory{i} =  fit(time_steps',trajectory{i}(:),'cubicinterp');
            end
            
        otherwise
            CASPR_log.Error('Path type is invalid');
    end
end

% A function to find out the common time range under all the conditions
function out = IntersectedTimeRange(range_1,range_2)

    out1(1:(numel(range_2)+(numel(range_1)-2)))=0;
    k=1;
    while isempty(range_1)==0 && isempty(range_2)==0
        % make sure that first is ahead second
        if range_1(1)>range_2(1)
            temp=range_2;
            range_2=range_1;
            range_1=temp;
        end
        if range_1(2)<range_2(1)
            range_1=range_1(3:end);
            continue;
        elseif range_1(2)==range_2(1)
            out1(k)=range_2(1);
            out1(k+1)=range_2(1);
            k=k+2;

            range_1=range_1(3:end);
            continue;
        else
            if range_1(2)==range_2(2)
                out1(k)=range_2(1);
                out1(k+1)=range_2(2);
                k=k+2;

                range_1=range_1(3:end);
                range_2=range_2(3:end);

            elseif range_1(2)<range_2(2)
                out1(k)=range_2(1);
                out1(k+1)=range_1(2);
                k=k+2;

                range_1=range_1(3:end);
            else
                out1(k)=range_2(1);
                out1(k+1)=range_2(2);
                k=k+2;

                range_2=range_2(3:end);
            end
        end
    end
    out=out1(1:k-1);
 end