% Represents a trajectory in joint space
%
% Author        : Darwin LAU
% Created       : 2014
% Description   :
classdef JointTrajectory < TrajectoryBase
    properties
        q           % The joint space coordinate as a cell array
        q_dot       % The joint space coordinate derivative as a cell array
        q_ddot      % The joint space coordinate double derivative as a cell array
    end
    
    methods
        % Function that samples the reference trajectory for frequency analysis later.
        % Should be called after the generation of the joint space
        % trajectory. Several options can be considered:
        %   1. Keep constant until the next trajectory point (ZOH)
        %       mode == 1
        %   2. Linearly interpolate two adjacent points
        %       mode == 2
        %   3. Keep the sampling rate the same as the trajectory's original
        %   sampling rate
        %       mode == 3
        function output_data = trajectorySampling(obj, working_mode, sampling_rate_ratio)
            % initialize the working mode and sampling frequency ratio
            mode    =   -1;
            sampling_frequency_ratio = 1;
            if (nargin < 3)
                CASPR_log.Info('Default frequency analysis will be applied (use the original trajectory frequency as the sampling rate).');
                mode = 3;
            else
                if working_mode == 1
                    mode = 1;
                    sampling_frequency_ratio = floor(sampling_rate_ratio);
                    if sampling_frequency_ratio < 1
                        sampling_frequency_ratio = 1;
                    end
                elseif working_mode == 2
                    mode = 2;
                    sampling_frequency_ratio = floor(sampling_rate_ratio);
                    if sampling_frequency_ratio < 1
                        sampling_frequency_ratio = 1;
                    end
                else
                    mode = 3;
                end
            end
            
            % generate the source signal for different modes
            traj_len = length(obj.q);
            traj_dim = length(obj.q{1});
            % Fs being the sampling frequency
            Fs      =   1/(obj.timeVector(2)-obj.timeVector(1));
            if mode == 1
                % ZOH approach
                q_source_sig        =   zeros(sampling_frequency_ratio*traj_len, traj_dim);
                q_dot_source_sig    =   zeros(sampling_frequency_ratio*traj_len, traj_dim);
                q_ddot_source_sig   =   zeros(sampling_frequency_ratio*traj_len, traj_dim);
                for i = 1:traj_len-1
                    for j = 1:sampling_frequency_ratio
                        q_source_sig(sampling_frequency_ratio*(i-1)+j, :) = obj.q{i}';
                        q_dot_source_sig(sampling_frequency_ratio*(i-1)+j, :) = obj.q_dot{i}';
                        q_ddot_source_sig(sampling_frequency_ratio*(i-1)+j, :) = obj.q_ddot{i}';
                    end
                end
                q_source_sig(sampling_frequency_ratio*(traj_len-1)+1, :) = obj.q{traj_len}';
                q_dot_source_sig(sampling_frequency_ratio*(traj_len-1)+1, :) = obj.q_dot{itraj_len}';
                q_ddot_source_sig(sampling_frequency_ratio*(traj_len-1)+1, :) = obj.q_ddot{traj_len}';
                time_vec_source_sig = obj.timeVector(1):sampling_frequency_ratio*(traj_len-1):obj.timeVector(length(obj.timeVector));
                % update the sampling frequency when necessary
                Fs = Fs*sampling_frequency_ratio;
            elseif mode == 2
                % linear interpolation approach
                q_source_sig        =   zeros(sampling_frequency_ratio*traj_len, traj_dim);
                q_dot_source_sig    =   zeros(sampling_frequency_ratio*traj_len, traj_dim);
                q_ddot_source_sig   =   zeros(sampling_frequency_ratio*traj_len, traj_dim);
                for i = 1:traj_len-1
                    q_interval      =   obj.q{i+1}' - obj.q{i}';
                    q_dot_interval  =   obj.q_dot{i+1}' - obj.q_dot{i}';
                    q_ddot_interval =   obj.q_ddot{i+1}' - obj.q_ddot{i}';
                    q_step          =   q_interval/sampling_frequency_ratio;
                    q_dot_step      =   q_dot_interval/sampling_frequency_ratio;
                    q_ddot_step     =   q_ddot_interval/sampling_frequency_ratio;
                    for j = 1:sampling_frequency_ratio
                        q_source_sig(sampling_frequency_ratio*(i-1)+j, :) = obj.q{i}' + (j-1)*q_step;
                        q_dot_source_sig(sampling_frequency_ratio*(i-1)+j, :) = obj.q_dot{i}' + (j-1)*q_dot_step;
                        q_ddot_source_sig(sampling_frequency_ratio*(i-1)+j, :) = obj.q_ddot{i}' + (j-1)*q_ddot_step;
                    end
                end
                q_source_sig(sampling_frequency_ratio*(traj_len-1)+1, :) = obj.q{traj_len}';
                q_dot_source_sig(sampling_frequency_ratio*(traj_len-1)+1, :) = obj.q_dot{itraj_len}';
                q_ddot_source_sig(sampling_frequency_ratio*(traj_len-1)+1, :) = obj.q_ddot{traj_len}';
                time_vec_source_sig = obj.timeVector(1):sampling_frequency_ratio*(traj_len-1):obj.timeVector(length(obj.timeVector));
                % update the sampling frequency when necessary
                Fs = Fs*sampling_frequency_ratio;
            elseif mode == 3
                q_source_sig        =   cell2mat(obj.q)';
                q_dot_source_sig    =   cell2mat(obj.q_dot)';
                q_ddot_source_sig   =   cell2mat(obj.q_ddot)';
                time_vec_source_sig =   obj.timeVector;
            else
                q_source_sig        =   cell2mat(obj.q)';
                q_dot_source_sig    =   cell2mat(obj.q_dot)';
                q_ddot_source_sig   =   cell2mat(obj.q_ddot)';
                time_vec_source_sig =   obj.timeVector;
            end
            
            output_data.q_sampled_data = q_source_sig;
            output_data.q_dot_sampled_data = q_dot_source_sig;
            output_data.q_ddot_sampled_data = q_ddot_source_sig;
            output_data.t_sampled_data = time_vec_source_sig;
            
            [output_data.q_fs_1, output_data.f] = fftAnalysis(output_data.t_sampled_data, output_data.q_sampled_data);
            [output_data.q_dot_fs_1, ~] = fftAnalysis(output_data.t_sampled_data, output_data.q_dot_sampled_data);
            [output_data.q_ddot_fs_1, ~] = fftAnalysis(output_data.t_sampled_data, output_data.q_ddot_sampled_data);
        end
        
        % Plots the joint space
        function plotJointSpace(obj, states_to_plot, plot_ax)
            obj.plotJointPose(states_to_plot, plot_ax);
            obj.plotJointVelocity(states_to_plot, plot_ax);
            obj.plotJointAcceleration(states_to_plot, plot_ax);
        end
        
        % Function plots the joint trajectory
        function plotJointPose(obj, states_to_plot, plot_ax)
            n_dof = length(obj.q{1});
            q_array = cell2mat(obj.q);
            
            if nargin <= 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            if(nargin <= 2 || isempty(plot_ax))
                % Plots joint space variables q(t)
                figure;
                plot(obj.timeVector, q_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                title('Joint Pose');
            else
                plot(plot_ax, obj.timeVector, q_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
            end
            xlabel('Time (s)');
            ylabel('Joint Pose');
        end
        
        % Function plots the joint trajectory velocity
        function plotJointVelocity(obj, states_to_plot, plot_ax)
            n_dof = length(obj.q_dot{1});
            qd_array = cell2mat(obj.q_dot);
            
            if nargin <= 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            if(nargin <= 2 || isempty(plot_ax))
                % Plots joint space variables q(t)
                figure;
                plot(obj.timeVector, qd_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                title('Joint velocity');
            else
                plot(plot_ax, obj.timeVector, qd_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
            end
            xlabel('Time (s)');
            ylabel('Joint Velocity');
        end
        
        % Function plots the joint trajectory, velocity and acceleration
        function plotJointAcceleration(obj, states_to_plot, plot_ax)
            n_dof = length(obj.q_ddot{1});
            qdd_array = cell2mat(obj.q_ddot);
            
            if nargin <= 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            if(nargin <= 2 || isempty(plot_ax))
                % Plots joint space variables q(t)
                figure;
                plot(obj.timeVector, qdd_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
                title('Joint acceleration');
            else
                plot(plot_ax, obj.timeVector, qdd_array(states_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
            end
            xlabel('Time (s)');
            ylabel('Joint Acceleration');
        end
        
        function saveJointTrajectoryFile(obj, file_name)
            n_q = length(obj.q{1});
            fid = fopen(file_name, 'w');
            fprintf(fid, 'time_step,%f\n', obj.timeStep);
            
            for i = 1:length(obj.timeVector)
                q_tmp = obj.q{i};
                v_tmp = obj.q_dot{i};
                a_tmp = obj.q_ddot{i};
                % First the time
                str = [sprintf('t,%0.6f,p,', obj.timeVector(i))];
                % Now add the joint pose
                for j = 1:n_q
                    str = [str, sprintf('%.6f,', q_tmp(j))];
                end
                str = [str,'v,'];
                for j = 1:n_q
                    str = [str, sprintf('%.6f,', v_tmp(j))];
                end
                str = [str,'a,'];
                for j = 1:n_q
                    if (j ~= n_q)
                        str = [str, sprintf('%.6f,', a_tmp(j))];
                    else
                        str = [str, sprintf('%.6f', a_tmp(j))];
                    end
                end
                if(i < length(obj.timeVector))
                    fprintf(fid,'%s\n',str);
                else
                    fprintf(fid,'%s',str);
                end
            end
            fclose(fid);
        end
    end
    
    methods (Static)
        % Loads all trajectories from XML configuration
        function [trajectory] = LoadXmlObj(xmlObj, bodiesObj, modelConfig)
            node_name = xmlObj.getNodeName;
            % First select the type of trajectory and then pass it to
            % individual functions
            if (strcmp(node_name, 'linear_spline_trajectory'))
                trajectory = JointTrajectory.LinearTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'cubic_spline_trajectory'))
                trajectory = JointTrajectory.CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'quintic_spline_trajectory'))
                trajectory = JointTrajectory.QuinticTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'cubic_spline_average_velocity_trajectory'))
                trajectory = JointTrajectory.CubicTrajectoryAverageVelocityLoadXmlObj(xmlObj, bodiesObj, modelConfig);
            elseif (strcmp(node_name, 'parabolic_blend_trajectory'))
                trajectory = JointTrajectory.ParabolicBlendTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig);
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
            q_pj = cell(num_points,1);
            time_points_abs = zeros(1, num_points);
            
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];

            
            % First process the data and save it to variables
            for p = 1:num_points
                point_node = point_nodes.item(p-1);
                q = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q').item(0).getFirstChild.getData));
                % Error checking on whether the XML file is valid
                CASPR_log.Assert(length(q) == bodiesObj.numDofVars, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofVars, length(q)));
                
                q_pj{p} = mat2cell(q, bodiesObj.jointsNumDofVars);
                
                if (p == 1)
                    time_points_abs(p) = 0.0;
                elseif (time_abs)
                    time_points_abs(p) = str2double(point_node.getAttribute('time'));
                else
                    time_points_abs(p) = time_points_abs(p-1) + str2double(point_node.getAttribute('time'));
                end
            end
                                   
            % Call the create function                 
            trajectory = JointTrajectory.LinearTrajectoryCreate(q_pj, time_points_abs, time_step, bodiesObj);
        end
        
        % Create the linear spline trajectory with given information
        function [trajectory] = LinearTrajectoryCreate(q_pj, time_points_abs, time_step, bodiesObj)
            trajectory = JointTrajectory;
            num_points = length(q_pj);
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];
            % Generate the trajectory between the points
            for p = 1:num_points-1
                q_section = [];
                q_d_section = [];
                q_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                for j = 1:bodiesObj.numLinks
                    try
                        [q_body, q_d_body, q_dd_body] = bodiesObj.bodies{j}.joint.generateTrajectoryLinearSpline(q_pj{p}{j}, q_pj{p+1}{j}, time_section);
                    catch
                        [q_body, q_d_body, q_dd_body] = bodiesObj.bodies{j}.joint.generateTrajectoryLinearSpline(q_pj{p}, q_pj{p+1}, time_section);
                    end
                    q_section = [q_section; q_body];
                    q_d_section = [q_d_section; q_d_body];
                    q_dd_section = [q_dd_section; q_dd_body];
                end
                if (p > 1)
                    q_section(:,1) = [];
                    q_d_section(:,1) = [];
                    q_dd_section(:,1) = [];
                    time_section(:,1) = [];
                end
                
                q_trajectory = [q_trajectory q_section];
                q_d_trajectory = [q_d_trajectory q_d_section];
                q_dd_trajectory = [q_dd_trajectory q_dd_section];
                trajectory.timeVector = [trajectory.timeVector time_section];
            end
            trajectory.q = mat2cell(q_trajectory, size(q_trajectory,1), ones(size(q_trajectory,2),1));
            trajectory.q_dot = mat2cell(q_d_trajectory, size(q_d_trajectory,1), ones(size(q_d_trajectory,2),1));
            trajectory.q_ddot = mat2cell(q_dd_trajectory, size(q_dd_trajectory,1), ones(size(q_dd_trajectory,2),1));
        end
        
        % Perform cubic trajectory spline to produce trajectory
        function [trajectory] = CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'cubic_spline_trajectory'), 'Element should be <cubic_spline_trajectory>');
            
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            time_abs = TrajectoryBase.get_xml_absolute_tag(xmlObj);
            
            % Cell of points of joints coordinates
            q_pj = cell(num_points,1);
            q_d_pj = cell(num_points,1);
            time_points_abs = zeros(1, num_points);            
            
            % First process the data and save it to variables
            for p = 1:num_points
                point_node = point_nodes.item(p-1);
                q = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q').item(0).getFirstChild.getData));
                q_d = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
                % Error checking on whether the XML file is valid
                CASPR_log.Assert(length(q) == bodiesObj.numDofVars, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofVars, length(q)));
                CASPR_log.Assert(length(q_d) == bodiesObj.numDofs, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofs, length(q_d)));
                
                q_pj{p} = mat2cell(q, bodiesObj.jointsNumDofVars);
                q_d_pj{p} = mat2cell(q_d, bodiesObj.jointsNumDofs);
                
                if (p == 1)
                    time_points_abs(p) = 0.0;
                elseif (time_abs)
                    time_points_abs(p) = str2double(point_node.getAttribute('time'));
                else
                    time_points_abs(p) = time_points_abs(p-1) + str2double(point_node.getAttribute('time'));
                end
            end
                                   
            % Call the create function                 
            trajectory = JointTrajectory.CubicTrajectoryCreate(q_pj, q_d_pj, time_points_abs, time_step, bodiesObj);
        end
        
        % Create the cubic spline trajectory with given information
        function [trajectory] = CubicTrajectoryCreate(q_pj, q_d_pj, time_points_abs, time_step, bodiesObj)
            trajectory = JointTrajectory;
            num_points = length(q_pj);
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];
            % Generate the trajectory between the points
            for p = 1:num_points-1
                q_section = [];
                q_d_section = [];
                q_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                for j = 1:bodiesObj.numLinks
                    [q_body, q_d_body, q_dd_body] = bodiesObj.bodies{j}.joint.generateTrajectoryCubicSpline(q_pj{p}{j}, q_d_pj{p}{j}, ...
                        q_pj{p+1}{j}, q_d_pj{p+1}{j}, time_section);
                    q_section = [q_section; q_body];
                    q_d_section = [q_d_section; q_d_body];
                    q_dd_section = [q_dd_section; q_dd_body];
                end
                if (p > 1)
                    q_section(:,1) = [];
                    q_d_section(:,1) = [];
                    q_dd_section(:,1) = [];
                    time_section(:,1) = [];
                end
                
                q_trajectory = [q_trajectory q_section];
                q_d_trajectory = [q_d_trajectory q_d_section];
                q_dd_trajectory = [q_dd_trajectory q_dd_section];
                trajectory.timeVector = [trajectory.timeVector time_section];
            end
            trajectory.q = mat2cell(q_trajectory, size(q_trajectory,1), ones(size(q_trajectory,2),1));
            trajectory.q_dot = mat2cell(q_d_trajectory, size(q_d_trajectory,1), ones(size(q_d_trajectory,2),1));
            trajectory.q_ddot = mat2cell(q_dd_trajectory, size(q_dd_trajectory,1), ones(size(q_dd_trajectory,2),1));
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
            q_pj = cell(num_points,1);
            q_d_pj = cell(num_points,1);
            q_dd_pj = cell(num_points,1);
            time_points_abs = zeros(1, num_points);            
            
            % First process the data and save it to variables
            for p = 1:num_points
                point_node = point_nodes.item(p-1);
                q = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q').item(0).getFirstChild.getData));
                q_d = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
                q_dd = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q_ddot').item(0).getFirstChild.getData));
                % Error checking on whether the XML file is valid
                CASPR_log.Assert(length(q) == bodiesObj.numDofVars, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofVars, length(q)));
                CASPR_log.Assert(length(q_d) == bodiesObj.numDofs, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofs, length(q_d)));
                CASPR_log.Assert(length(q_dd) == bodiesObj.numDofs, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofs, length(q_dd)));
                
                q_pj{p} = mat2cell(q, bodiesObj.jointsNumDofVars);
                q_d_pj{p} = mat2cell(q_d, bodiesObj.jointsNumDofs);
                q_dd_pj{p} = mat2cell(q_dd, bodiesObj.jointsNumDofs);
                
                if (p == 1)
                    time_points_abs(p) = 0.0;
                elseif (time_abs)
                    time_points_abs(p) = str2double(point_node.getAttribute('time'));
                else
                    time_points_abs(p) = time_points_abs(p-1) + str2double(point_node.getAttribute('time'));
                end
            end
                                   
            % Call the create function                 
            trajectory = JointTrajectory.QuinticTrajectoryCreate(q_pj, q_d_pj, q_dd_pj, ...
                time_points_abs, time_step, bodiesObj);
        end        
        
        % Create the quintic spline trajectory with given information
        function [trajectory] = QuinticTrajectoryCreate(q_pj, q_d_pj, q_dd_pj, time_points_abs, time_step, bodiesObj)
            trajectory = JointTrajectory;
            num_points = length(q_pj);
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];
            
            % Generate the trajectory between the points
            for p = 1:num_points-1
                q_section = [];
                q_d_section = [];
                q_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                for j = 1:bodiesObj.numLinks
                    [q_body, q_d_body, q_dd_body] = bodiesObj.bodies{j}.joint.generateTrajectoryQuinticSpline(q_pj{p}{j}, q_d_pj{p}{j}, q_dd_pj{p}{j}, ...
                        q_pj{p+1}{j}, q_d_pj{p+1}{j}, q_dd_pj{p+1}{j}, time_section);
                    q_section = [q_section; q_body];
                    q_d_section = [q_d_section; q_d_body];
                    q_dd_section = [q_dd_section; q_dd_body];
                end
                if (p > 1)
                    q_section(:,1) = [];
                    q_d_section(:,1) = [];
                    q_dd_section(:,1) = [];
                    time_section(:,1) = [];
                end
                
                q_trajectory = [q_trajectory q_section];
                q_d_trajectory = [q_d_trajectory q_d_section];
                q_dd_trajectory = [q_dd_trajectory q_dd_section];
                trajectory.timeVector = [trajectory.timeVector time_section];
            end
            trajectory.q = mat2cell(q_trajectory, size(q_trajectory,1), ones(size(q_trajectory,2),1));
            trajectory.q_dot = mat2cell(q_d_trajectory, size(q_d_trajectory,1), ones(size(q_d_trajectory,2),1));
            trajectory.q_ddot = mat2cell(q_dd_trajectory, size(q_dd_trajectory,1), ones(size(q_dd_trajectory,2),1));
        end
        
        function [trajectory] = CubicTrajectoryAverageVelocityLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'cubic_spline_average_velocity_trajectory'), 'Element should be <cubic_spline_average_velocity_trajectory>');
            
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            time_abs = TrajectoryBase.get_xml_absolute_tag(xmlObj);
            
            % Cell of points of joints coordinates
            q_pj = cell(num_points,1);
            q_d_pj = cell(num_points,1);
            time_points_abs = zeros(1, num_points);            
            
            % First process the data and save it to variables
            for p = 1:num_points
                point_node = point_nodes.item(p-1);
                q = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q').item(0).getFirstChild.getData));
                % Error checking on whether the XML file is valid
                CASPR_log.Assert(length(q) == bodiesObj.numDofVars, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofVars, length(q)));
                
                q_pj{p} = mat2cell(q, bodiesObj.jointsNumDofVars);
                
                if (p == 1)
                    time_points_abs(p) = 0.0;
                elseif (time_abs)
                    time_points_abs(p) = str2double(point_node.getAttribute('time'));
                else
                    time_points_abs(p) = time_points_abs(p-1) + str2double(point_node.getAttribute('time'));
                end
            end
            
            % Determine the average velocities
            for p = 1:num_points
                if p > 1 && p < num_points
                    vel1 = (cell2mat(q_pj{p})-cell2mat(q_pj{p-1}))/(time_points_abs(p)-time_points_abs(p-1));
                    vel2 = (cell2mat(q_pj{p+1})-cell2mat(q_pj{p}))/(time_points_abs(p+1)-time_points_abs(p));
                    q_d = (vel1+vel2)/2;
                else
                    point_node = point_nodes.item(p-1);
                    CASPR_log.Assert(~isempty(point_node.getElementsByTagName('q_dot').item(0)), 'Initial and final velocities must be specified');
                    q_d = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
                    CASPR_log.Assert(length(q_d) == bodiesObj.numDofs, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofs, length(q_d)));
                end
                q_d_pj{p} = mat2cell(q_d, bodiesObj.jointsNumDofs);
            end
            
            % Call the create function                 
            trajectory = JointTrajectory.CubicTrajectoryAverageVelocityCreate(q_pj, q_d_pj, time_points_abs, time_step, bodiesObj);
        end       
        
        % Create the cubic trajectory (average velocity) with given information
        function [trajectory] = CubicTrajectoryAverageVelocityCreate(q_pj, q_d_pj, time_points_abs, time_step, bodiesObj)
            trajectory = JointTrajectory;
            num_points = length(q_pj);
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];
            
            % Generate the trajectory between the points
            for p = 1:num_points-1
                q_section = [];
                q_d_section = [];
                q_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                
                for j = 1:bodiesObj.numLinks
                    [q_body, q_d_body, q_dd_body] = bodiesObj.bodies{j}.joint.generateTrajectoryCubicSpline(q_pj{p}{j}, q_d_pj{p}{j}, ...
                        q_pj{p+1}{j}, q_d_pj{p+1}{j}, time_section);
                    q_section = [q_section; q_body];
                    q_d_section = [q_d_section; q_d_body];
                    q_dd_section = [q_dd_section; q_dd_body];
                end
                if (p > 1)
                    q_section(:,1) = [];
                    q_d_section(:,1) = [];
                    q_dd_section(:,1) = [];
                    time_section(:,1) = [];
                end
                
                q_trajectory = [q_trajectory q_section];
                q_d_trajectory = [q_d_trajectory q_d_section];
                q_dd_trajectory = [q_dd_trajectory q_dd_section];
                trajectory.timeVector = [trajectory.timeVector time_section];
            end
            trajectory.q = mat2cell(q_trajectory, size(q_trajectory,1), ones(size(q_trajectory,2),1));
            trajectory.q_dot = mat2cell(q_d_trajectory, size(q_d_trajectory,1), ones(size(q_d_trajectory,2),1));
            trajectory.q_ddot = mat2cell(q_dd_trajectory, size(q_dd_trajectory,1), ones(size(q_dd_trajectory,2),1));
        end
        
        function [trajectory] = ParabolicBlendTrajectoryLoadXmlObj(xmlObj, bodiesObj, ~)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'parabolic_blend_trajectory'), 'Element should be <parabolic_blend_trajectory>');
           
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            time_blend_default = str2double(xmlObj.getAttribute('blend_time_default'));
            
            time_abs = TrajectoryBase.get_xml_absolute_tag(xmlObj);
            
            % Cell of points of joints coordinates
            q_pj = cell(num_points,1);
            q_d_pj = cell(num_points,1);       
            time_points_abs = zeros(1, num_points);
            time_blend = time_blend_default*ones(1, num_points-1);           
            
            % First process the data and save it to variables
            for p = 1:num_points
                point_node = point_nodes.item(p-1);
                q = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q').item(0).getFirstChild.getData));
                if (num_points > 1 && ~isempty(char(point_node.getAttribute('blend_time'))))
                    time_blend(p-1) = str2double(char(point_node.getAttribute('blend_time')));
                end
                
                % Error checking on whether the XML file is valid
                CASPR_log.Assert(length(q) == bodiesObj.numDofVars, sprintf('Trajectory config point does not contain correct number of variables, desired : %d, specified : %d', bodiesObj.numDofVars, length(q)));
                
                q_pj{p} = mat2cell(q, bodiesObj.jointsNumDofVars);
                
                if (p == 1)
                    time_points_abs(p) = 0.0;
                elseif (time_abs)
                    time_points_abs(p) = str2double(point_node.getAttribute('time'));
                else
                    time_points_abs(p) = time_points_abs(p-1) + str2double(point_node.getAttribute('time'));
                end
            end

                                   
            % Call the create function                 
            trajectory = JointTrajectory.ParabolicBlendTrajectoryCreate(q_pj, ...
                time_points_abs, time_step, time_blend, bodiesObj);
        end       
        
        % Create the parabolic blend trajectory with given information
        function [trajectory] = ParabolicBlendTrajectoryCreate(q_pj, time_points_abs, time_step, time_blend, bodiesObj)
            trajectory = JointTrajectory;
            num_points = length(q_pj);
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];
            
            % Generate the trajectory between the points
            for p = 1:num_points-1
                t_rel = time_points_abs(p+1)-time_points_abs(p);
                CASPR_log.Assert(time_blend(p)*2 < t_rel, 'Time between two sections must be larger than two times the blend time');
                q_section = [];
                q_d_section = [];
                q_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                for j = 1:bodiesObj.numLinks
                    [q_body, q_d_body, q_dd_body] = bodiesObj.bodies{j}.joint.generateTrajectoryParabolicBlend(q_pj{p}{j}, q_pj{p+1}{j}, ...
                        time_section, time_blend(p));
                    q_section = [q_section; q_body];
                    q_d_section = [q_d_section; q_d_body];
                    q_dd_section = [q_dd_section; q_dd_body];
                end
                if (p > 1)
                    q_section(:,1) = [];
                    q_d_section(:,1) = [];
                    q_dd_section(:,1) = [];
                    time_section(:,1) = [];
                end
                
                q_trajectory = [q_trajectory q_section];
                q_d_trajectory = [q_d_trajectory q_d_section];
                q_dd_trajectory = [q_dd_trajectory q_dd_section];
                trajectory.timeVector = [trajectory.timeVector time_section];
            end
            trajectory.q = mat2cell(q_trajectory, size(q_trajectory,1), ones(size(q_trajectory,2),1));
            trajectory.q_dot = mat2cell(q_d_trajectory, size(q_d_trajectory,1), ones(size(q_d_trajectory,2),1));
            trajectory.q_ddot = mat2cell(q_dd_trajectory, size(q_dd_trajectory,1), ones(size(q_dd_trajectory,2),1));
        end
        
        
        function [trajectory] = FileTrajectoryLoadXmlObj(xmlObj, bodiesObj, modelConfig)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'file_trajectory'), 'Element should be <file_trajectory>');
           
            filename = char(xmlObj.getAttribute('filename'));
            file_location = [modelConfig.modelFolderPath, filename];
            
            trajectory = JointTrajectory.FileTrajectoryCreate(file_location, bodiesObj);
        end
        
        % Loads a complete trajectory by reading a .traj file
        function [trajectory] = FileTrajectoryCreate(traj_file, model)
            % Initialise a new trajectry
            trajectory = JointTrajectory;
            
            % Read through the trajectory file
            % Open the file
            fid = fopen(traj_file, 'r');
            % Read the first line
            l0 = fgetl(fid);
            % Split the line
            l_split = strsplit(l0,'time_step,');
            % extract the double values
            timeStep = str2double(l_split{2});
            
            time = 0.0;
            
            % For the remaining lines extract the data
            num_points = 1;
            % Initialise the trajectory vectors
            n_q = model.numDofs;
            while(~feof(fid))
                line = fgetl(fid);
                % Split up the components
                l_split = strsplit(line,{'t,','p,',',v,',',a,'});
                
                % Extract and check the time (error checking that file is
                % valid)
                l_time = str2double(l_split{2});
                % Check that time is within the mantissa
                CASPR_log.Assert(abs(time - l_time) <= 1e-9, 'Invalid format: time is not correct');
                time = time + timeStep;
                
                % First the pose
                l_split_p = strsplit(l_split{3},',');
                q = zeros(n_q,1);
                CASPR_log.Assert(n_q == length(l_split_p), 'Invalid format: invalid number of joint poses specified');
                for k=1:length(l_split_p)
                    q(k) = str2double(l_split_p{k});
                end
                trajectory.q{num_points} = q;
                
                % Then the velocity
                l_split_v = strsplit(l_split{4},',');
                v = zeros(n_q,1);
                CASPR_log.Assert(n_q == length(l_split_v), 'Invalid format: invalid number of joint velocities specified');
                for k=1:length(l_split_v)
                    v(k) = str2double(l_split_v{k});
                end
                trajectory.q_dot{num_points} = v;
                
                % Then the acceleration
                l_split_a = strsplit(l_split{5},',');
                a = zeros(n_q,1);
                CASPR_log.Assert(n_q == length(l_split_a), 'Invalid format: invalid number of joint accelerations specified');
                for k=1:length(l_split_a)
                    a(k) = str2double(l_split_a{k});
                end
                trajectory.q_ddot{num_points} = a;
                
                num_points = num_points + 1;
            end
            totalTime = (num_points-2)*timeStep;
            trajectory.timeVector = [0:timeStep:totalTime];
            fclose(fid);
        end
        
        % NEEDS TO BE FIXED TO GENERATE BASED ON JOINTS AND MERGE WITH
        % EXISTING PARABOLIC BLEND
        
        % The arguments time_blend_s and time_blend_e can be only used to
        % decide the acceleration of the triangular/trapezoidal profile.
        function [trajectory] = ParabolicBlendTrajectoryGenerate(q_s, q_e, time_step, time_blend_s, time_blend_e, v_max)
            CASPR_log.Assert(length(q_s) == length(q_e), 'Length of input states are inconsistent!');
            if(q_s == q_e)
                trajectory.q = q_s;
                trajectory.q_dot = q_s*0;
                trajectory.q_ddot = q_s*0;
                trajectory.timeVector = 0;
                return;
            end
            n_dof = length(q_s);
            vmax = abs(v_max);
            delta_q = q_e-q_s;
            distance = norm(delta_q);
            
            if(1/2*vmax*(time_blend_s+time_blend_e)>=distance)
                % Triangular Profile
                acc_s = vmax/time_blend_s;
                acc_e = vmax/time_blend_e;
                if(time_blend_s == 0)
                    time_acc_s = 0;
                    time_acc_e = ceil(sqrt(2*distance/acc_e)/time_step)*time_step;
                elseif(time_blend_e == 0)
                    time_acc_s = ceil(sqrt(2*distance/acc_s)/time_step)*time_step;
                    time_acc_e = 0;
                else
                    time_acc_s = ceil(sqrt(2*acc_e*distance/acc_s/(acc_s+acc_e))/time_step)*time_step;
                    time_acc_e = ceil(sqrt(2*acc_s*distance/acc_e/(acc_s+acc_e))/time_step)*time_step;
                end
                time_const_speed = 0;
            else
                % Trapezoidal Profile
                time_acc_s = ceil(time_blend_s/time_step)*time_step;
                time_acc_e = ceil(time_blend_e/time_step)*time_step;
                distance_const_speed = distance - (vmax*(time_acc_s+time_acc_e)/2);
                if(distance_const_speed<=0)
                    time_const_speed = 0;
                else
                    time_const_speed = ceil(distance_const_speed/vmax/time_step)*time_step;
                end
            end
            time_vector = 0 : time_step : time_acc_s+time_acc_e+time_const_speed;
            
            q = zeros(n_dof, length(time_vector));
            q_dot = zeros(n_dof, length(time_vector));
            q_ddot = zeros(n_dof, length(time_vector));
            
            v_max_true = delta_q/(1/2*(time_acc_s+time_acc_e)+time_const_speed);            
            acc_true_s = v_max_true/time_acc_s;
            acc_true_e = v_max_true/time_acc_e;
            for t_ind = 1:length(time_vector)
                t = time_vector(t_ind);
                if (t < time_acc_s)
                    q(:,t_ind) = q_s + acc_true_s*t^2/2;
                    q_dot(:,t_ind) = acc_true_s*t;
                    q_ddot(:,t_ind) = acc_true_s;
                elseif (t <= time_acc_s + time_const_speed)
                    q(:,t_ind) = q_s + v_max_true*time_acc_s/2 + v_max_true * (t-time_acc_s);
                    q_dot(:,t_ind) = v_max_true;
                    q_ddot(:,t_ind) = 0;
                else
                    q(:,t_ind) = -acc_true_e*(t-time_acc_s-time_const_speed-time_acc_e)^2/2 + q_e;
                    q_dot(:,t_ind) = -acc_true_e*(t-time_acc_s-time_const_speed-time_acc_e);
                    q_ddot(:,t_ind) = -acc_true_e;
                end
            end
            %             trajectory.q = q;
            %             trajectory.q_dot = q_dot;
            %             trajectory.q_ddot = q_ddot;
            %             trajectory.timeVector = time_vector;
            
            % The purpose to add the start and end points without velocity
            % and acceleration is to make sure the end effector is still
            % when it is ready to go or stops.
            % Doing this is a little unreasonable, especially for the start
            % point due to the replicated time 0, however, we can use this
            % to debug, and anyway, the added end point is worth keeping.
            trajectory.q = [q_s,q,q_e];
            trajectory.q_dot = [q_s*0,q_dot,q_e*0];
            trajectory.q_ddot = [q_s*0,q_ddot,q_e*0];
            trajectory.timeVector = [-time_step, time_vector, time_vector(end)+time_step];
        end
    end
end