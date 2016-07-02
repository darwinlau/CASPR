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
        function trajectory = LoadXmlObj(xmlObj, bodiesObj)
            CASPR_log.Assert(strcmp(xmlObj.getNodeName, 'trajectory'), 'Element should be <trajectory>');
            total_time = str2double(xmlObj.getElementsByTagName('time_total').item(0).getFirstChild.getData);
            time_step = str2double(xmlObj.getElementsByTagName('time_step').item(0).getFirstChild.getData);
            beginObj = xmlObj.getElementsByTagName('begin').item(0);
            endObj = xmlObj.getElementsByTagName('end').item(0);
            
            q_s = XmlOperations.StringToVector(char(beginObj.getElementsByTagName('q').item(0).getFirstChild.getData));
            q_s_d = XmlOperations.StringToVector(char(beginObj.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
            q_s_dd = XmlOperations.StringToVector(char(beginObj.getElementsByTagName('q_ddot').item(0).getFirstChild.getData));
            q_e = XmlOperations.StringToVector(char(endObj.getElementsByTagName('q').item(0).getFirstChild.getData));
            q_e_d = XmlOperations.StringToVector(char(endObj.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
            q_e_dd = XmlOperations.StringToVector(char(endObj.getElementsByTagName('q_ddot').item(0).getFirstChild.getData));
            
            % Error checking on whether the XML file is valid
            CASPR_log.Assert(length(q_s) == bodiesObj.numDofVars, sprintf('Trajectory config does not contain correct number of DoF vars for q begin, desired : %d, specified : %d', bodiesObj.numDofVars, length(q_s)));
            CASPR_log.Assert(length(q_s_d) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_dot begin, desired : %d, specified : %d', bodiesObj.numDofs, length(q_s_d)));
            CASPR_log.Assert(length(q_s_dd) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_ddot begin, desired : %d, specified : %d', bodiesObj.numDofs, length(q_s_dd)));
            CASPR_log.Assert(length(q_e) == bodiesObj.numDofVars, sprintf('Trajectory config does not contain correct number of DoF vars for q end, desired : %d, specified : %d', bodiesObj.numDofVars, length(q_e)));
            CASPR_log.Assert(length(q_e_d) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_dot end, desired : %d, specified : %d', bodiesObj.numDofs, length(q_e_d)));
            CASPR_log.Assert(length(q_e_dd) == bodiesObj.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_ddot end, desired : %d, specified : %d', bodiesObj.numDofs, length(q_e_dd)));
            
            trajectory = JointTrajectory.GenerateTrajectory(bodiesObj, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step);
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