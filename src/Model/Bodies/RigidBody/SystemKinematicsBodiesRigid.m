classdef SystemKinematicsBodiesRigid < SystemKinematicsBodies
    %BODYSYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    methods 
        function bk = SystemKinematicsBodiesRigid(bodyPropFilepath)
            % Load the cable properties into the CableKinematics list
            assert(exist(bodyPropFilepath, 'file') == 2, 'Bodies property file does not exist');
            fid = fopen(bodyPropFilepath);
            
            % read line one "num_cables, num_links"
            line = fgetl(fid);
            s = sscanf(line,'%d,%d');
            num_links = s(1);
            num_dofs = s(2);
            
            bk@SystemKinematicsBodies(num_links, num_dofs);
            
            num_dofs = 0;
            try                
                k = 0;
                % read each cable line
                while ~feof(fid)
                    k = k+1;
                    % Set ID as link number
                    
                    line = fgetl(fid);
                    line_entries = regexp(line, ',', 'split');
                    joint_type_string = line_entries(1);
                    bk.bodies{k} = BodyKinematics(k, JointType.(joint_type_string{1}));
                    
                    % Dynamics properties
%                         mass = str2double(line_entries(2));
%                         Ixx = str2double(line_entries(3));
%                         Iyy = str2double(line_entries(4));
%                         Izz = str2double(line_entries(5));
%                         Ixy = str2double(line_entries(6));
%                         Ixz = str2double(line_entries(7));
%                         Iyz = str2double(line_entries(8));
                    
                    % Kinematics properties
                    bk.bodies{k}.r_G = [str2double(line_entries(9)); str2double(line_entries(10)); str2double(line_entries(11))];
                    bk.bodies{k}.r_P = [str2double(line_entries(12)); str2double(line_entries(13)); str2double(line_entries(14))];
                    
                    parent_link_num = str2double(line_entries(15));
                    r_parent_loc = [str2double(line_entries(16)); str2double(line_entries(17)); str2double(line_entries(18))];
                    
                    bk.connectBodies(parent_link_num, k, r_parent_loc);
                    num_dofs = num_dofs + bk.bodies{k}.joint.numDofs;
                end % end while
            catch err
                fclose(fid);
                if (strcmp(err.identifier, 'MATLAB:subscripting:classHasNoPropertyOrMethod'))
                    error('Joint type not defined in enum');
                end % end if
                error('Invalid format for body property config');
            end
            
            fclose(fid);
            assert(num_dofs == bk.numDofs, 'Number of DoFs specified and number of DoFs added are inconsistent');
            
            % Do an update with zero positions
            bk.update(zeros(bk.numDofs,1), zeros(bk.numDofs,1), zeros(bk.numDofs,1));
        end
        
        function update(obj, q, q_dot, q_ddot)
            % Can update states (such as local attachments) first if necessary
            % Since bodies and attachment locations do not change, no need to update anything here
            
            % Update standard things from kinematics
            update@SystemKinematicsBodies(obj, q, q_dot, q_ddot);
            
            % Post updates if necessary
        end
    end
    
end

