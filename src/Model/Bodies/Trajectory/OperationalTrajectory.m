% Represents a trajectory in operational space
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OperationalTrajectory < TrajectoryBase
    properties
        y           % The op coordinate as a cell array
        y_dot       % The op coordinate derivative as a cell array
        y_ddot      % The op coordinate double derivative as a cell array
    end
    
    methods        
        % Plots the operational space
        function plotOperationalSpace(obj, states_to_plot)
            n_operational_dof = length(obj.y{1});
            
            if nargin == 1 || isempty(states_to_plot)
                states_to_plot = 1:n_operational_dof;
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
    
    methods (Static)
        % Loads all trajectories from XML configuration
        function [trajectory] = LoadXmlObj(xmlObj, bodiesObj)
            node_name = xmlObj.getNodeName;
            % First select the type of trajectory and then pass it to 
            % individual functions
            if (strcmp(node_name, 'linear_spline_trajectory'))
                trajectory = OperationalTrajectory.LinearTrajectoryLoadXmlObj(xmlObj, bodiesObj);
            elseif (strcmp(node_name, 'cubic_spline_trajectory'))
                trajectory = OperationalTrajectory.CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj);
            elseif (strcmp(node_name, 'quintic_spline_trajectory'))
                trajectory = OperationalTrajectory.QuinticTrajectoryLoadXmlObj(xmlObj, bodiesObj);
            elseif (strcmp(node_name, 'cubic_spline_average_velocity_trajectory'))
                trajectory = OperationalTrajectory.CubicTrajectoryAverageVelocityLoadXmlObj(xmlObj, bodiesObj);
            elseif (strcmp(node_name, 'parabolic_blend_trajectory'))
                trajectory = OperationalTrajectory.ParabolicBlendTrajectoryLoadXmlObj(xmlObj, bodiesObj);
            else
                CASPR_log.Error('Trajectory type in XML undefined');
            end
        end
        
        % Perform linear trajectory spline to produce trajectory
        function [trajectory] = LinearTrajectoryLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end
        
        % Perform quintic trajectory spline to produce trajectory
        function [trajectory] = CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end
        
        % Perform quintic trajectory spline to produce trajectory
        function [trajectory] = QuinticTrajectoryLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end        
        
        function [trajectory] = CubicTrajectoryAverageVelocityLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end       
        
        function [trajectory] = ParabolicBlendTrajectoryLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end       
        
        % Loads a complete trajectory by reading a .traj file
        function [trajectory_all, force_trajectory] = LoadCompleteTrajectory(traj_file,model)
            CASPR_log.Error('Function has not been implemented yet');
        end
    end
end