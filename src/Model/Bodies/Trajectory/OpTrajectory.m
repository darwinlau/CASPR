classdef OpTrajectory < handle
    %TRAJECTORYINTERPOLATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        y
        y_dot
        y_ddot
        timeVector
        totalTime
        timeStep
    end
    
    methods
        function op = OpTrajectory(y_begin,yd_begin,ydd_begin,y_end,yd_end,ydd_end, total_time, time_step)
            % Save the time information
            op.timeStep         =   time_step;
            op.totalTime        =   total_time;
            op.timeVector       =   0:time_step:total_time;
            % May need to change later
            [op.y,op.y_dot,op.y_ddot]    =   Spline.QuinticInterpolation(y_begin,yd_begin,ydd_begin,y_end,yd_end,ydd_end,op.timeVector);
        end
        
        function plotOpSpace(obj, states_to_plot)
            n_op_dof = length(obj.y{1});
            
            if nargin == 1 || isempty(states_to_plot)
                states_to_plot = 1:n_op_dof;
            end
            
            y_vector = cell2mat(obj.q);
            yd_vector = cell2mat(obj.q_dot);
            ydd_vector = cell2mat(obj.q_ddot);
            
            figure;
            title('Trajectory');
            plot(obj.timeVector, y_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
            
            figure;
            title('Trajectory velocity');
            plot(obj.timeVector, yd_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
            
            figure;
            title('Trajectory acceleration');
            plot(obj.timeVector, ydd_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
        end
    end
end