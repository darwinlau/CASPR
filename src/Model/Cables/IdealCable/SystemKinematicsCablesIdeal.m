classdef SystemKinematicsCablesIdeal < SystemKinematicsCables
    %CABLEKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
        
    methods
        function ck = SystemKinematicsCablesIdeal(cablePropFilepath)
            % Load the cable properties into the CableKinematics list
            assert(exist(cablePropFilepath, 'file') == 2, 'Cable property file does not exist');
            fid = fopen(cablePropFilepath);
            
            % read line one "num_cables, num_links"
            line = fgetl(fid);
            s = sscanf(line, '%d,%d');
            
            num_cables = s(1);
            num_links = s(2);
            ck@SystemKinematicsCables(num_cables, num_links);
            
            try 
                i = 0;
                while ~feof(fid)
                    i = i+1;
                    line = fgetl(fid);
                    line_entries = regexp(line, ',', 'split');
                    
                    % Entry 1 is cable name
                    ck.cables{i}.name = line_entries(1);
                    % Entries 2, 3, 4 are cable dynamics properties
                    % Kinematics start at 5
                    k = 5;
                    % Link number, X, Y, Z : 4 values
                    att_ind_count = 4;
                    
                    while (k + 2*att_ind_count - 1 <= length(line_entries) && ~isnan(str2double(line_entries(k))) && ~isnan(str2double(line_entries(k+att_ind_count))))
                        % Read from file
                        sLink = str2double(line_entries(k));
                        sLoc = str2double([line_entries(k+1);line_entries(k+2);line_entries(k+3)]);
                        nLink = str2double(line_entries(k+att_ind_count));
                        nLoc = str2double([line_entries(k+att_ind_count+1);line_entries(k+att_ind_count+2);line_entries(k+att_ind_count+3)]);
                        % Add the segment
                        ck.cables{i}.addSegment(sLink, sLoc, nLink, nLoc);
                        k = k+att_ind_count;
                    end
                end
            catch err
                fclose(fid);
                error('Invalid format for cable property config');
            end
            fclose(fid);
            
            assert(i == ck.numCables, sprintf('Cable config does not contain correct number of cables, specified : %d, added : %d', ck.numCables, i));
        end
        
        function update(obj, bodyKinematics)
            % Can update states (such as local attachments) first if necessary
            % Since all local attachments do not change, no need to update anything here
            
            % Update standard things from kinematics
            update@SystemKinematicsCables(obj, bodyKinematics);
            
            % Post updates if necessary
        end
    end
end

