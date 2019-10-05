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
        function plotOperationalSpace(obj, states_to_plot, plot_ax)
            obj.plotOperationalPose(states_to_plot, plot_ax);
            obj.plotOperationalVelocity(states_to_plot, plot_ax);
            obj.plotOperationalAcceleration(states_to_plot, plot_ax);
        end
        
        function plotOperationalPose(obj, states_to_plot, plot_ax)
            n_dof = length(obj.y{1});
            y_array = cell2mat(obj.y);
            
            if nargin <= 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            if(nargin <= 2 || isempty(plot_ax))
                % Plots joint space variables q(t)
                figure;
                plot(obj.timeVector, y_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                title('Operational Pose');
            else
                plot(plot_ax, obj.timeVector, y_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
            end
            xlabel('Time (s)');
            ylabel('Operational Pose');
        end
        
        function plotOperationalVelocity(obj, states_to_plot, plot_ax)
            n_dof = length(obj.y_dot{1});
            y_dot_array = cell2mat(obj.y_dot);
            
            if nargin <= 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            if(nargin <= 2 || isempty(plot_ax))
                % Plots joint space variables q(t)
                figure;
                plot(obj.timeVector, y_dot_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                title('Operational Velocity');
            else
                plot(plot_ax, obj.timeVector, y_dot_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
            end
            xlabel('Time (s)');
            ylabel('Operational Velocity');
        end
        
        function plotOperationalAcceleration(obj, states_to_plot, plot_ax)
            n_dof = length(obj.y_ddot{1});
            y_dot_array = cell2mat(obj.y_ddot);
            
            if nargin <= 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            if(nargin <= 2 || isempty(plot_ax))
                % Plots joint space variables q(t)
                figure;
                plot(obj.timeVector, y_dot_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                title('Operational Acceleration');
            else
                plot(plot_ax, obj.timeVector, y_dot_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
            end
            xlabel('Time (s)');
            ylabel('Operational Acceleration');
        end
    end
    
    methods (Static)
        % Loads all trajectories from XML configuration
        function [trajectory] = LoadXmlObj(xmlObj, bodiesObj, modelConfig)
            node_name = xmlObj.getNodeName;
            % First select the type of trajectory and then pass it to 
            % individual functions
            if (strcmp(node_name, 'linear_spline_trajectory'))
                trajectory = OperationalTrajectory.LinearTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'cubic_spline_trajectory'))
                trajectory = OperationalTrajectory.CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'quintic_spline_trajectory'))
                trajectory = OperationalTrajectory.QuinticTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'cubic_spline_average_velocity_trajectory'))
                trajectory = OperationalTrajectory.CubicTrajectoryAverageVelocityLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'parabolic_blend_trajectory'))
                trajectory = OperationalTrajectory.ParabolicBlendTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'file_trajectory'))
                trajectory = JointTrajectory.FileTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            else
                CASPR_log.Error('Trajectory type in XML undefined');
            end
        end
        
        % Perform linear trajectory spline to produce trajectory
        function [trajectory] = LinearTrajectoryLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'linear_spline_trajectory'), 'Element should be <linear_spline_trajectory>');            
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            time_abs = TrajectoryBase.get_xml_absolute_tag(xmlObj);
                  
            % Cell of points of joints coordinates
            y_pj = cell(num_points,1); 
            time_points_abs = zeros(1, num_points);          
            
            % First process the data and save it to variables
            for p = 1:num_points
                point_node = point_nodes.item(p-1);
                y = XmlOperations.StringToVector(char(point_node.getElementsByTagName('y').item(0).getFirstChild.getData));
                % Error checking on whether the XML file is valid
                CASPR_log.Assert(length(y) == bodiesObj.numOperationalDofs, sprintf('Trajectory config point does not contain correct number of operational space variables, desired : %d, specified : %d', bodiesObj.numOperationalDofs, length(y)));
                               
                y_pj{p} = mat2cell(y, bodiesObj.operationalSpaceNumDofs);
                
                if (p == 1)
                    time_points_abs(p) = 0.0;
                elseif (time_abs)
                    time_points_abs(p) = str2double(point_node.getAttribute('time'));
                else
                    time_points_abs(p) = time_points_abs(p-1) + str2double(point_node.getAttribute('time'));
                end
            end
            % Call the create function                 
            trajectory = OperationalTrajectory.LinearTrajectoryCreate(y_pj, time_points_abs, time_step, bodiesObj);
        end
        
        % Create the linear spline trajectory with given information
        function [trajectory] = LinearTrajectoryCreate(y_pj, time_points_abs, time_step, bodiesObj)
            trajectory = OperationalTrajectory;
            num_points = length(y_pj);
            y_trajectory = [];
            y_d_trajectory = [];
            y_dd_trajectory = [];
            % Generate the trajectory between the points
            for p = 1:num_points-1
                y_section = [];
                y_d_section = [];
                y_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                for j = 1:bodiesObj.numOperationalSpaces
                    [y_op, y_d_op, y_dd_op] = bodiesObj.bodies{bodiesObj.operationalSpaceBodyIndices(j)}.operationalSpace.generateTrajectoryLinearSpline(y_pj{p}{j}, y_pj{p+1}{j}, time_section);
                    y_section = [y_section; y_op];
                    y_d_section = [y_d_section; y_d_op];
                    y_dd_section = [y_dd_section; y_dd_op];
                end
                if (p > 1)
                    y_section(:,1) = [];
                    y_d_section(:,1) = [];
                    y_dd_section(:,1) = [];
                    time_section(:,1) = [];
                end
                
                y_trajectory = [y_trajectory y_section];
                y_d_trajectory = [y_d_trajectory y_d_section];
                y_dd_trajectory = [y_dd_trajectory y_dd_section];
                trajectory.timeVector = [trajectory.timeVector time_section];
            end            
            trajectory.y = mat2cell(y_trajectory, size(y_trajectory,1), ones(size(y_trajectory,2),1));
            trajectory.y_dot = mat2cell(y_d_trajectory, size(y_d_trajectory,1), ones(size(y_d_trajectory,2),1));
            trajectory.y_ddot = mat2cell(y_dd_trajectory, size(y_dd_trajectory,1), ones(size(y_dd_trajectory,2),1));
        end
        
        % Perform cubic trajectory spline to produce trajectory
        function [trajectory] = CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Error('Function has not been implemented yet');
        end
        
        % Create the cubic spline trajectory with given information
        function [trajectory] = CubicTrajectoryCreate(num_points, y_pj, y_d_pj, y_dd_pj, time_points_abs, time_step, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end
        
        % Perform quintic trajectory spline to produce trajectory
        function [trajectory] = QuinticTrajectoryLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'quintic_spline_trajectory'), 'Element should be <quintic_spline_trajectory>');
            
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            time_abs = TrajectoryBase.get_xml_absolute_tag(xmlObj);
                  
            % Cell of points of joints coordinates
            y_pj = cell(num_points,1); 
            y_d_pj = cell(num_points,1);
            y_dd_pj = cell(num_points,1);
            time_points_abs = zeros(1, num_points);            
            
            % First process the data and save it to variables
            for p = 1:num_points
                point_node = point_nodes.item(p-1);
                y = XmlOperations.StringToVector(char(point_node.getElementsByTagName('y').item(0).getFirstChild.getData));
                y_d = XmlOperations.StringToVector(char(point_node.getElementsByTagName('y_dot').item(0).getFirstChild.getData));
                y_dd = XmlOperations.StringToVector(char(point_node.getElementsByTagName('y_ddot').item(0).getFirstChild.getData));
                % Error checking on whether the XML file is valid
                CASPR_log.Assert(length(y) == bodiesObj.numOperationalDofs, sprintf('Trajectory config point does not contain correct number of operational space variables, desired : %d, specified : %d', bodiesObj.numOperationalDofs, length(y)));
                CASPR_log.Assert(length(y_d) == bodiesObj.numOperationalDofs, sprintf('Trajectory config point does not contain correct number of operational space variables, desired : %d, specified : %d', bodiesObj.numOperationalDofs, length(y_d)));
                CASPR_log.Assert(length(y_dd) == bodiesObj.numOperationalDofs, sprintf('Trajectory config point does not contain correct number of operational space variables, desired : %d, specified : %d', bodiesObj.numOperationalDofs, length(y_dd)));
                               
                y_pj{p} = mat2cell(y, bodiesObj.operationalSpaceNumDofs);
                y_d_pj{p} = mat2cell(y_d, bodiesObj.operationalSpaceNumDofs);
                y_dd_pj{p} = mat2cell(y_dd, bodiesObj.operationalSpaceNumDofs);
                
                if (p == 1)
                    time_points_abs(p) = 0.0;
                elseif (time_abs)
                    time_points_abs(p) = str2double(point_node.getAttribute('time'));
                else
                    time_points_abs(p) = time_points_abs(p-1) + str2double(point_node.getAttribute('time'));
                end
            end
                                   
            % Call the create function                 
            trajectory = OperationalTrajectory.QuinticTrajectoryCreate(y_pj, y_d_pj, y_dd_pj, ...
                time_points_abs, time_step, bodiesObj);
        end     
        
        % Create the quintic spline trajectory with given information
        function [trajectory] = QuinticTrajectoryCreate(y_pj, y_d_pj, y_dd_pj, time_points_abs, time_step, bodiesObj)
            trajectory = OperationalTrajectory;
            num_points = length(y_pj);
            y_trajectory = [];
            y_d_trajectory = [];
            y_dd_trajectory = [];
            % Generate the trajectory between the points
            for p = 1:num_points-1
                y_section = [];
                y_d_section = [];
                y_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                for j = 1:bodiesObj.numOperationalSpaces
                    [y_op, y_d_op, y_dd_op] = bodiesObj.bodies{bodiesObj.operationalSpaceBodyIndices(j)}.operationalSpace.generateTrajectoryQuinticSpline(y_pj{p}{j}, y_d_pj{p}{j}, y_dd_pj{p}{j}, y_pj{p+1}{j}, y_d_pj{p+1}{j}, y_dd_pj{p+1}{j}, time_section);
                    
                    y_section = [y_section; y_op];
                    y_d_section = [y_d_section; y_d_op];
                    y_dd_section = [y_dd_section; y_dd_op];
                end
                if (p > 1)
                    y_section(:,1) = [];
                    y_d_section(:,1) = [];
                    y_dd_section(:,1) = [];
                    time_section(:,1) = [];
                end
                
                y_trajectory = [y_trajectory y_section];
                y_d_trajectory = [y_d_trajectory y_d_section];
                y_dd_trajectory = [y_dd_trajectory y_dd_section];
                trajectory.timeVector = [trajectory.timeVector time_section];
            end            
            trajectory.y = mat2cell(y_trajectory, size(y_trajectory,1), ones(size(y_trajectory,2),1));
            trajectory.y_dot = mat2cell(y_d_trajectory, size(y_d_trajectory,1), ones(size(y_d_trajectory,2),1));
            trajectory.y_ddot = mat2cell(y_dd_trajectory, size(y_dd_trajectory,1), ones(size(y_dd_trajectory,2),1));
        end
        
        function [trajectory] = CubicTrajectoryAverageVelocityLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Error('Function has not been implemented yet');
        end  
        
        function [trajectory] = CubicTrajectoryAverageVelocityCreate(num_points, y_pj, y_d_pj, y_dd_pj, time_points_abs, time_step, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end 
        
        function [trajectory] = ParabolicBlendTrajectoryLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Error('Function has not been implemented yet');
        end   
        
        function [trajectory] = ParabolicBlendTrajectoryCreate(num_points, y_pj, y_d_pj, y_dd_pj, time_points_abs, time_step, bodiesObj)
            CASPR_log.Error('Function has not been implemented yet');
        end 
        
        function [trajectory] = FileTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig)
            CASPR_log.Error('Function has not been implemented yet');
        end   
    end
end