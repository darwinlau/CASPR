% Library of file management utilities
%
% Author        : Jonathan EDEN
% Created       : 2014
% Description   :
classdef FileOperations
    methods(Static)
        % A function to convert the reference trajectory into a traj file
        % (file_name).
        function CompleteTrajectoryGenerator(refTrajectory, file_name, operation_mode)
            t = refTrajectory.timeVector;
            n_q = length(refTrajectory.q{1});
            fid = fopen(file_name,operation_mode);
            for i = 1:length(t)-1
                q_tmp = refTrajectory.q{i};
                v_tmp = refTrajectory.q_dot{i};
                a_tmp = refTrajectory.q_ddot{i};
                % First the time
                str = ['t,',num2str(t(i)),',q,'];
                % Now add the joint pose
                for j = 1:n_q
                    str = [str,num2str(q_tmp(j)),','];
                end
                str = [str,'v,'];
                for j = 1:n_q
                    str = [str,num2str(v_tmp(j)),','];
                end
                str = [str,'a,'];
                for j = 1:n_q
                    str = [str,num2str(a_tmp(j)),','];
                end
                if(i ~= length(t)-1)
                    fprintf(fid,'%s\n',str);
                else
                    fprintf(fid,'%s',str);
                end
            end
            fclose(fid);
        end
        
        % A function to convert the reference operational trajectory into a traj file
        % (file_name).
        function CompleteOperationalTrajectoryGenerator(refTrajectory, file_name, operation_mode)
            t = refTrajectory.timeVector;
            n_y = length(refTrajectory.y{1});
            fid = fopen(file_name,operation_mode);
            for i = 1:length(t)-1
                y_tmp = refTrajectory.y{i};
                v_tmp = refTrajectory.y_dot{i};
                a_tmp = refTrajectory.y_ddot{i};
                % First the time
                str = ['t,',num2str(t(i)),',y,'];
                % Now add the operational pose
                for j = 1:n_y
                    str = [str,num2str(y_tmp(j)),','];
                end
                str = [str,'v,'];
                for j = 1:n_y
                    str = [str,num2str(v_tmp(j)),','];
                end
                str = [str,'a,'];
                for j = 1:n_y
                    str = [str,num2str(a_tmp(j)),','];
                end
                if(i ~= length(t)-1)
                    fprintf(fid,'%s\n',str);
                else
                    fprintf(fid,'%s',str);
                end
            end
            fclose(fid);
        end
        
        % A function to convert a traj file (file_name) into a trajectory.
        function trajectory = ReadCompleteTrajectory(file_name,n_q)
            % File 
            fid = fopen(file_name,'r');
            % Initialise a new trajectry
            trajectory = JointTrajectory;
            
            % For the remaining lines extract the data
            num_points = 1; 
            while(~feof(fid))
                l1 = fgetl(fid);
                % Split the components around the time 
                length_string = length(l1);
                t_split = strsplit(l1,{'t,'});
                CASPR_log.Assert(length(t_split{2})==length_string-2,'t should be the first entry of each trajectory line');
                % Split the outcome using q
                l_split_q = strsplit(t_split{2},',q,');
                % Test if t was a single entry
                CASPR_log.Assert(length(strsplit(l_split_q{1},',')) == 1,'t should be the first entry of each trajectory line');
                % Save t
                trajectory.timeVector{num_points} = str2double(l_split_q{1});
                % Test if there were multiple q variables
                CASPR_log.Assert(isempty(strfind(l_split_q{2},'q')),'Only one q should occur on each line');
                % Split the outcome using v
                l_split_v = strsplit(l_split_q{2},',v,');
                pose_split = strsplit(l_split_v{1},',');
                CASPR_log.Assert(length(pose_split)==n_q,'Invalid number of joint poses specified');
                q = zeros(n_q,1);
                for k=1:length(pose_split)
                    q(k) = str2double(pose_split{k});
                end
                trajectory.q{num_points} = q;
                % Test if there were multiple v variables
                CASPR_log.Assert(isempty(strfind(l_split_v{2},'v')),'Only one v should occur on each line');
                % Split the outcome using a
                l_split_a = strsplit(l_split_v{2},',a,');
                vel_split = strsplit(l_split_a{1},',');
                CASPR_log.Assert(length(vel_split)==n_q,'Invalid number of joint velocities specified');
                v = zeros(n_q,1);
                for k=1:length(vel_split)
                    v(k) = str2double(vel_split{k});
                end
                trajectory.q_dot{num_points} = v;
                % Test if there were multiple a variables
                CASPR_log.Assert(isempty(strfind(l_split_a{2},'a')),'Only one a should occur on each line');
                acc_split = strsplit(l_split_a{2},',');
                CASPR_log.Assert(length(acc_split)==n_q+1,'Invalid number of joint accelerations specified');
                a = zeros(n_q,1);
                for k=1:length(acc_split)-1
                    a(k) = str2double(acc_split{k});
                end
                trajectory.q_ddot{num_points} = a;               
                
                num_points = num_points + 1;
            end
            fclose(fid);
        end
        
        % A function to convert a traj file (file_name) into a trajectory.
        function trajectory = ReadCompleteOperationalTrajectory(file_name,n_y)
            % File 
            fid = fopen(file_name,'r');
            % Initialise a new trajectry
            trajectory = OperationalTrajectory;
            
            % For the remaining lines extract the data
            num_points = 1; 
            while(~feof(fid))
                l1 = fgetl(fid);
                % Split the components around the time 
                length_string = length(l1);
                t_split = strsplit(l1,{'t,'});
                CASPR_log.Assert(length(t_split{2})==length_string-2,'t should be the first entry of each trajectory line');
                % Split the outcome using q
                l_split_y = strsplit(t_split{2},',y,');
                % Test if t was a single entry
                CASPR_log.Assert(length(strsplit(l_split_y{1},',')) == 1,'t should be the first entry of each trajectory line');
                % Save t
                trajectory.timeVector{num_points} = str2double(l_split_y{1});
                % Test if there were multiple q variables
                CASPR_log.Assert(isempty(strfind(l_split_y{2},'q')),'Only one y should occur on each line');
                % Split the outcome using v
                l_split_v = strsplit(l_split_y{2},',v,');
                pose_split = strsplit(l_split_v{1},',');
                CASPR_log.Assert(length(pose_split)==n_y,'Invalid number of joint poses specified');
                y = zeros(n_y,1);
                for k=1:length(pose_split)
                    y(k) = str2double(pose_split{k});
                end
                trajectory.y{num_points} = y;
                % Test if there were multiple v variables
                CASPR_log.Assert(isempty(strfind(l_split_v{2},'v')),'Only one v should occur on each line');
                % Split the outcome using a
                l_split_a = strsplit(l_split_v{2},',a,');
                vel_split = strsplit(l_split_a{1},',');
                CASPR_log.Assert(length(vel_split)==n_y,'Invalid number of joint velocities specified');
                v = zeros(n_y,1);
                for k=1:length(vel_split)
                    v(k) = str2double(vel_split{k});
                end
                trajectory.y_dot{num_points} = v;
                % Test if there were multiple a variables
                CASPR_log.Assert(isempty(strfind(l_split_a{2},'a')),'Only one a should occur on each line');
                acc_split = strsplit(l_split_a{2},',');
                CASPR_log.Assert(length(acc_split)==n_y+1,'Invalid number of joint accelerations specified');
                a = zeros(n_y,1);
                for k=1:length(acc_split)-1
                    a(k) = str2double(acc_split{k});
                end
                trajectory.y_ddot{num_points} = a;               
                
                num_points = num_points + 1;
            end
            trajectory.timeVector = cell2mat(trajectory.timeVector);
            fclose(fid);
        end
    end
end