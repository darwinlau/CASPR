classdef SystemDynamicsCablesIdeal < SystemDynamicsCables
    %CABLEKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
        
    methods
        function cd = SystemDynamicsCablesIdeal(cablePropFilepath)
            % Load the cable properties into the CableKinematics list
            assert(exist(cablePropFilepath, 'file') == 2, 'Cable property file does not exist');
            fid = fopen(cablePropFilepath);
            
            % read line one "num_cables, num_links"
            line = fgetl(fid);
            s = sscanf(line, '%d,%d');
            
            num_cables = s(1);
            cd@SystemDynamicsCables(num_cables);
            
            try 
                i = 0;
                while ~feof(fid)
                    i = i+1;
                    line = fgetl(fid);
                    line_entries = regexp(line, ',', 'split');
                    % Entry 1 is cable name
                    cd.cables{i}.name = line_entries(1);
                    % Entries 2, 3, 4 are cable dynamics properties
                    fmin = str2double(line_entries(2));
                    fmax = str2double(line_entries(3));
                    ferror = str2double(line_entries(4));
                    
                    cd.cables{i}.forceMin = fmin;
                    cd.cables{i}.forceMax = fmax; 
                    cd.cables{i}.forceInvalid = ferror;
                end
            catch err
                fclose(fid);
                error('Invalid format for cable property config');
            end
            fclose(fid);
            
            assert(i == cd.numCables, sprintf('Cable config does not contain correct number of cables, specified : %d, added : %d', cd.numCables, i));
        end
        function update(obj, cableKinematics, bodyKinematics, bodyDynamics)
            % Can update states (such as cable min/max forces) first if necessary
            
            % Update standard things from kinematics
            update@SystemDynamicsCables(obj, cableKinematics, bodyKinematics, bodyDynamics)
            
            % Post updates if necessary
        end
    end
end

