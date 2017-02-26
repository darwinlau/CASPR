% Represents a trajectory in joint space
%
% Author        : Darwin LAU
% Created       : 2014
% Description   :
classdef JointTrajectory < handle
    properties
        q           % The joint space coordinate as a cell array
        q_dot       % The joint space coordinate derivative as a cell array
        q_ddot      % The joint space coordinate double derivative as a cell array
        timeVector  % The time vector
    end
    
    properties (Dependent)
        totalTime 
    end
    
    methods
        % Function plots the joint trajectory, velocity and acceleration
        function plotJointSpace(obj, states_to_plot)
            n_dof = length(obj.q{1});
            
            if nargin == 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            
            q_vector = cell2mat(obj.q);
            qd_vector = cell2mat(obj.q_dot);
            qdd_vector = cell2mat(obj.q_ddot);
            
            figure;
            title('Trajectory');
            plot(obj.timeVector, q_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
            
            figure;
            title('Trajectory velocity');
            plot(obj.timeVector, qd_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
            
            figure;
            title('Trajectory acceleration');
            plot(obj.timeVector, qdd_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
        end
        
        function value = get.totalTime(obj)
            value = obj.timeVector(length(obj.timeVector)) - obj.timeVector(1);
        end
    end
    
    methods (Static)
        % Loads all trajectories from XML configuration
        function [trajectory] = LoadXmlObj(xmlObj, bodiesObj)
            node_name = xmlObj.getNodeName;
            % First select the type of trajectory and then pass it to 
            % individual functions
            if (strcmp(node_name, 'linear_spline_trajectory'))
                trajectory = JointTrajectory.LinearTrajectoryLoadXmlObj(xmlObj, bodiesObj);
            elseif (strcmp(node_name, 'cubic_spline_trajectory'))
                trajectory = JointTrajectory.CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj);
            elseif (strcmp(node_name, 'quintic_spline_trajectory'))
                trajectory = JointTrajectory.QuinticTrajectoryLoadXmlObj(xmlObj, bodiesObj);
            else
                CASPR_log.Error('Trajectory type in XML undefined');
            end
        end
                
        % Perform linear trajectory spline to produce trajectory
        function [trajectory] = LinearTrajectoryLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'linear_spline_trajectory'), 'Element should be <linear_spline_trajectory>');
            trajectory = JointTrajectory;
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            time_abs = JointTrajectory.get_xml_absolute_tag(xmlObj);
                  
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
                                   
            % Generate the trajectory between the points
            for p = 1:num_points-1
                q_section = [];
                q_d_section = [];
                q_dd_section = [];
                time_section = time_points_abs(p):time_step:time_points_abs(p+1);
                for j = 1:bodiesObj.numLinks
                    [q_body, q_d_body, q_dd_body] = bodiesObj.bodies{j}.joint.generateTrajectoryLinearSpline(q_pj{p}{j}, q_pj{p+1}{j}, time_section);
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
        function [trajectory] = CubicTrajectoryLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'cubic_spline_trajectory'), 'Element should be <cubic_spline_trajectory>');
            trajectory = JointTrajectory;
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            time_abs = JointTrajectory.get_xml_absolute_tag(xmlObj);
                  
            % Cell of points of joints coordinates
            q_pj = cell(num_points,1); 
            q_d_pj = cell(num_points,1);
            time_points_abs = zeros(1, num_points);
            
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];
            
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
        function [trajectory] = QuinticTrajectoryLoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'quintic_spline_trajectory'), 'Element should be <quintic_spline_trajectory>');
            trajectory = JointTrajectory;
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            time_abs = JointTrajectory.get_xml_absolute_tag(xmlObj);
                  
            % Cell of points of joints coordinates
            q_pj = cell(num_points,1); 
            q_d_pj = cell(num_points,1);
            q_dd_pj = cell(num_points,1);
            time_points_abs = zeros(1, num_points);
            
            q_trajectory = [];
            q_d_trajectory = [];
            q_dd_trajectory = [];
            
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
        
        % Loads a complete trajectory by reading a .traj file
        function [trajectory_all, force_trajectory] = LoadCompleteTrajectory(traj_file,model)
            % Initialise a new trajectry
            trajectory_all = JointTrajectory();
            
            % Read through the trajectory file            
            % Open the file
            fid = fopen(traj_file,'r');
            % Read the first line
            l0 = fgetl(fid);
            % Split the line
            l_split = strsplit(l0,'t,');
            % extract the double values
            trajectory_all.timeStep = str2double(l_split{2});
            
            % For the remaining lines extract the data
            num_points = 1;
            % Initialise the trajectory vectors
            n_q = model.bodyModel.numDofs; n_f = model.cableModel.numCables;
            while(~feof(fid))
                l1 = fgetl(fid);
                % Split up the components
                l_split = strsplit(l1,{'q,',',v,',',a,',',f,'});
                
                % First the pose
                l_split_q = strsplit(l_split{2},',');
                q = zeros(n_q,1);
                for k=1:length(l_split_q)
                    q(k) = str2double(l_split_q{k});
                end
                trajectory_all.q{num_points} = q;
                
                % Then the velocity
                l_split_v = strsplit(l_split{3},',');
                v = zeros(n_q,1);
                for k=1:length(l_split_v)
                    v(k) = str2double(l_split_v{k});
                end
                trajectory_all.q_dot{num_points} = v;
                
                % Then the acceleration
                l_split_a = strsplit(l_split{4},',');
                a = zeros(n_q,1);
                for k=1:length(l_split_a)
                    a(k) = str2double(l_split_a{k});
                end
                trajectory_all.q_ddot{num_points} = a;
                
                % Finally the force
                l_split_f = strsplit(l_split{5},',');
                f = zeros(n_f,1);
                for k=1:length(l_split_f)-1
                    f(k) = str2double(l_split_f{k});
                end
                force_trajectory{num_points} = f;
                
                num_points = num_points + 1;
            end
            trajectory_all.totalTime = (num_points-2)*trajectory_all.timeStep;
            trajectory_all.timeVector = [0:trajectory_all.timeStep:trajectory_all.totalTime];            
            fclose(fid);
        end
    end
    
    methods (Static, Access=private)
        function time_abs = get_xml_absolute_tag(xmlobj)
            time_def_str = xmlobj.getAttribute('time_definition');
            time_abs = 0;
            
            if (strcmp(time_def_str, 'relative'))
                time_abs = 0;
            elseif (strcmp(time_def_str, 'absolute'))
                time_abs = 1;
            else
                CASPR_log.Error('Value of attribute ''time_definition'' must either be ''relative'' or ''absolute''');
            end
        end
    end
end