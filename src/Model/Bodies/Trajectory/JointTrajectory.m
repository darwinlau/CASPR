% Represents a trajectory in joint space
%
% Author        : Darwin LAU
% Created       : 2014
% Description   :
classdef JointTrajectory < handle
    
    properties
        q           % The joint space coordinate
        q_dot       % The joint space coordinate derivative
        q_ddot      % The joint space coordinate double derivative
        timeVector  % The time vector
        totalTime   % The total time step
        timeStep    % The time step
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
    end
    
    methods (Static)
        % Loads trajectory from XML configuration
        function [trajectory_all] = LoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'trajectory'), 'Element should be <trajectory>');
            points_node = xmlObj.getElementsByTagName('points').item(0);
            point_nodes = points_node.getChildNodes;
            num_points = point_nodes.getLength;
            time_step = str2double(xmlObj.getAttribute('time_step'));
            
            q_points = cell(num_points,1);
            q_points_d = cell(num_points,1);
            q_points_dd = cell(num_points,1);
            time_points = zeros(1, num_points);
            
            for k = 1:num_points
                point_node = point_nodes.item(k-1);
                q_points{k} = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q').item(0).getFirstChild.getData));
                q_points_d{k} = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
                q_points_dd{k} = XmlOperations.StringToVector(char(point_node.getElementsByTagName('q_ddot').item(0).getFirstChild.getData));
                if (k > 1)
                    time_points(k) = str2double(point_node.getAttribute('time'));
                end
    %             % Error checking on whether the XML file is valid
    %             CASPR_log.Assert(length(q_s) == bodiesObj.numDofVars, sprintf('Trajectory config does not contain correct number of DoF vars for q begin, desired : %d, specified : %d', bodiesObj.numDofVars, length(q_s)));
    %             CASPR_log.Assert(length(q_s_d) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_dot begin, desired : %d, specified : %d', bodiesObj.numDofs, length(q_s_d)));
    %             CASPR_log.Assert(length(q_s_dd) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_ddot begin, desired : %d, specified : %d', bodiesObj.numDofs, length(q_s_dd)));
    %             CASPR_log.Assert(length(q_e) == bodiesObj.numDofVars, sprintf('Trajectory config does not contain correct number of DoF vars for q end, desired : %d, specified : %d', bodiesObj.numDofVars, length(q_e)));
    %             CASPR_log.Assert(length(q_e_d) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_dot end, desired : %d, specified : %d', bodiesObj.numDofs, length(q_e_d)));
    %             CASPR_log.Assert(length(q_e_dd) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_ddot end, desired : %d, specified : %d', bodiesObj.numDofs, length(q_e_dd)));
    %             
            end
            
            for k = 1:num_points-1
                trajectory = JointTrajectory.GenerateTrajectory(bodiesObj, q_points{k}, q_points_d{k}, q_points_dd{k}, q_points{k+1}, q_points_d{k+1}, q_points_dd{k+1}, time_points(k+1)-time_points(k), time_step);
                a = trajectory.q(1:length(trajectory.q));
                b = trajectory.q_dot(1:length(trajectory.q_dot));
                c = trajectory.q_ddot(1:length(trajectory.q_ddot));
                t = trajectory.timeVector(1:length(trajectory.timeVector));
                
                if (k < 2)
                new_q = a;
                new_q_dot = b;
                new_q_ddot = c;
                new_timeVector = t;
               
                end
                if(k > 1)
                a(1) = [];
                b(1) = [];
                c(1) = [];
                t(1) = [];
                t = t + new_timeVector(end);
                new_q = horzcat(new_q, a);
                new_q_dot = horzcat(new_q_dot, b);
                new_q_ddot = horzcat(new_q_ddot, c);
                new_timeVector = horzcat(new_timeVector, t);
                end
            end
            trajectory_all = JointTrajectory();
            trajectory_all.q = new_q;
            trajectory_all.q_dot = new_q_dot;
            trajectory_all.q_ddot = new_q_ddot;
            trajectory_all.timeVector = new_timeVector;
            trajectory_all.totalTime = trajectory_all.timeVector(end);
            trajectory_all.timeStep = time_step;
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
        
    
        % Generates trajectory from the starting and ending joint poses for
        % the entire system. Calls the generate trajectory function of each
        % type of joint.
        function trajectory = GenerateTrajectory(bodiesObj, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
            trajectory = JointTrajectory;
            n_dof = bodiesObj.numDofs;    
            t = 0:time_step:total_time;
            trajectory.timeVector = t;
            trajectory.totalTime = total_time;
            trajectory.timeStep = time_step;
                     
            q_array = zeros(n_dof, length(t));
            qd_array = zeros(n_dof, length(t));
            qdd_array = zeros(n_dof, length(t));
            
            index_dof = 1;
            index_var = 1;
            for k = 1:bodiesObj.numLinks
                ind_k_s_dof = index_dof;
                ind_k_e_dof = index_dof+bodiesObj.bodies{k}.joint.numDofs-1;
                ind_k_s_var = index_var;
                ind_k_e_var = index_var+bodiesObj.bodies{k}.joint.numVars-1;
                qk_s = q_s(ind_k_s_var:ind_k_e_var);
                qk_e = q_e(ind_k_s_var:ind_k_e_var);
                qk_s_d = q_s_d(ind_k_s_dof:ind_k_e_dof);
                qk_e_d = q_e_d(ind_k_s_dof:ind_k_e_dof);
                qk_s_dd = q_s_dd(ind_k_s_dof:ind_k_e_dof);
                qk_e_dd = q_e_dd(ind_k_s_dof:ind_k_e_dof);
                
                [qk, qk_dot, qk_ddot] = bodiesObj.bodies{k}.joint.GenerateTrajectory(qk_s, qk_s_d, qk_s_dd, qk_e, qk_e_d, qk_e_dd, total_time, time_step);

                q_array(ind_k_s_var:ind_k_e_var, :) = qk;
                qd_array(ind_k_s_dof:ind_k_e_dof, :) = qk_dot;
                qdd_array(ind_k_s_dof:ind_k_e_dof, :) = qk_ddot;
                index_dof = index_dof+bodiesObj.bodies{k}.joint.numDofs;
                index_var = index_var+bodiesObj.bodies{k}.joint.numVars;
            end
            s_q = size(q_array);
            s_q_d = size(qd_array);
            
            trajectory.q = mat2cell(q_array, s_q(1), ones(1, s_q(2)));
            trajectory.q_dot = mat2cell(qd_array, s_q_d(1), ones(1, s_q_d(2)));
            trajectory.q_ddot = mat2cell(qdd_array, s_q_d(1), ones(1, s_q_d(2)));
        end
    end
end