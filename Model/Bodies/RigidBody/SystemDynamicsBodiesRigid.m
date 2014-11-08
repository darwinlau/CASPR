classdef SystemDynamicsBodiesRigid < SystemDynamicsBodies
    %BODYSYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    methods 
        function bd = SystemDynamicsBodiesRigid(bodyPropFilepath)
            % Load the cable properties into the CableKinematics list
            assert(exist(bodyPropFilepath, 'file') == 2, 'Bodies property file does not exist');
            fid = fopen(bodyPropFilepath);
            
            % read line one "num_cables, num_links"
            line = fgetl(fid);
            s = sscanf(line,'%d,%d');
            num_links = s(1);
            num_dofs = s(2);
            
            bd@SystemDynamicsBodies(num_links, num_dofs);
            
            num_dofs = 0;
            try                
                k = 0;
                % read each cable line
                while ~feof(fid)
                    k = k+1;
                    % Set ID as link number
                    
                    line = fgetl(fid);
                    line_entries = regexp(line, ',', 'split');
                    bd.bodies{k} = BodyDynamics(k);
                    
                    % Dynamics properties
                    mass = str2double(line_entries(2));
                    Ixx = str2double(line_entries(3));
                    Iyy = str2double(line_entries(4));
                    Izz = str2double(line_entries(5));
                    Ixy = str2double(line_entries(6));
                    Ixz = str2double(line_entries(7));
                    Iyz = str2double(line_entries(8));
                    
                    bd.bodies{k}.m = mass;
                    bd.bodies{k}.I_G = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
                end % end while
            catch err
                fclose(fid);
                error('Invalid format for body property config');
            end
            
            fclose(fid);
            
            % Do an update with zero positions
            bd.update(SystemKinematicsBodiesRigid(bodyPropFilepath));
        end
        
        function update(obj, bodyKinematics)
            % Can update states (such as local attachments) first if necessary
            % Since bodies and attachment locations do not change, no need to update anything here
            
            % Update standard things from kinematics
            update@SystemDynamicsBodies(obj, bodyKinematics);
            
            % Post updates if necessary
        end
    end
    
end

