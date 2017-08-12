classdef FileOperation < handle
    properties (Access = private)
        % This file is used to record the initial position of the
        % dynamixels and the number of the brick which is being picked up
        % or being placed.
        filepath_record
        
        % This file is for outputting the coordinate of the brick waiting
        % to be picked up.
        filepath_pickup_co
        
        % This file is for outputting the coordinate of the brick where it
        % should be placed.
        filepath_place_co
        
        % To specify which brick is being processed.
        brick_number
        
        % size: 8 by 1
        % meaning: the initial position of 8 motors.
        init_position
        
        % Contail all the coordinate of the pick-up bricks.
        % With a structure of 3 by N(the number of the bricks)
        % For each column of this variable, (x;y;z;theta) specifies the
        % relevant pose of the brick.
        pickup_list
        
        % The meaning of each element is as same as the pickup_list. So
        % this variable should have the same size as the pickup_list
        place_list
    end
    
    properties (Access = private, Constant = true)
        length_hand = 0.224; % length of the hand
        length_end  = 0.180; % length of the other part of the hand
        length_offset = -0.00; % half of the height of the end effector
        length_insert = 0.030;
        vertical_offset = FileOperation.length_hand + FileOperation.length_end + FileOperation.length_offset - FileOperation.length_insert;
    end
    
    methods
        function fo = FileOperation(filepath_record, filepath_pickup_co, filepath_place_co)
            fo.filepath_record = filepath_record;
            if(nargin>1 && exist('filepath_pickup_co','var') && exist('filepath_place_co','var'))
                fo.filepath_pickup_co = filepath_pickup_co;
                fo.filepath_place_co = filepath_place_co;
                fo.readCoFile();
            end
        end
        
        function [pickup_co, place_co]= getCoordinate(obj, num)
            pickup_co = obj.pickup_list(num,1:3)'/1000;
            pickup_co(1) = 4-pickup_co(1)-0.015;
            pickup_co(2) = pickup_co(2)+0.006;
            pickup_co(3) = pickup_co(3)/0.051*0.051375 + obj.vertical_offset-0.0;
            
            place_co = obj.place_list(num,1:3)'/1000;
            place_co(1) = 4-place_co(1)+0.0;
            place_co(3) = place_co(3)/0.051*0.051375 + obj.vertical_offset+0.010; %+0.002+0.005;
        end
        
        function [arm_angle_pickup, arm_angle_place] = getArmAngle(obj,num)
            arm_angle_pickup = obj.pickup_list(num,4);
            arm_angle_place = 180-obj.place_list(num,4);
        end
        
        function [init_pos] = readInitPos_Motors(obj)
            fileID = fopen(obj.filepath_record,'r');
            formatSpec = '%f';
            data = fscanf(fileID,formatSpec);
            fclose(fileID);
            init_pos = data(1:8);
        end
        
        function writeInitPos_Motors(obj, data_pos)
            fileID = fopen(obj.filepath_record,'r');
            formatSpec = '%f';
            data = fscanf(fileID,formatSpec);
            fclose(fileID);
            
            data(1:8) = data_pos;
            
            fileID = fopen(obj.filepath_record,'w');
            fprintf(fileID,'%f \n',data);
            fclose(fileID);
        end
        
        function [num] = readBrickNum(obj)
            fileID = fopen(obj.filepath_record,'r');
            formatSpec = '%f';
            data = fscanf(fileID,formatSpec);
            fclose(fileID);
            num = data(9);
        end
        
        function writeBrickNum(obj, num)
            fileID = fopen(obj.filepath_record,'r');
            formatSpec = '%f';
            data = fscanf(fileID,formatSpec);
            fclose(fileID);
            
            if(length(data)==9)
                data(9) = num;
            elseif length(data)==8
                data = [data;num];
            else
                CASPR_log.Error('The format of the Record File is not incorrect!');
            end
            
            fileID = fopen(obj.filepath_record,'w');
            fprintf(fileID,'%f \n',data);
            fclose(fileID);
        end
        
        % Get the number of the brick involed for the present construction.
        function [num] = getAllBrickCount(obj)
            num = min(size(obj.place_list,1),size(obj.pickup_list,1));
        end
    end
    
    methods (Access = private)
        function readCoFile(obj)
            %             fileID = fopen(obj.filepath_pickup_co,'r');
            %             formatSpec = '%f %f %f %f';
            %             size_pickup_list = [4 Inf];
            %             obj.pickup_list = fscanf(fileID, formatSpec, size_pickup_list);
            %             fclose(fileID);
            obj.pickup_list = load(obj.filepath_pickup_co);
            %obj.pickup_list = flip(obj.pickup_list);
            %
            %             fileID = fopen(obj.filepath_place_co,'r');
            %             formatSpec = '%f %f %f %f';
            %             size_place_list = [4 Inf];
            %             obj.place_list = fscanf(fileID, formatSpec, size_place_list);
            %             fclose(fileID);
            obj.place_list = load(obj.filepath_place_co);
            %obj.place_list = flip(obj.place_list);
%             CASPR_log.Assert(size(obj.pickup_list) == size(obj.place_list), ...
%                 'The coordinate files of the picking up and placing do not match each other!');
        end
    end
    
    methods(Static)
        function recordData(data)
            filename = ['record' datestr(now,'yyyymmdd') '.csv'];
            [pathstr,~,~] = fileparts(mfilename('fullpath'));
            fullname = [pathstr '\Application\Record\' filename];
            
            dlmwrite(fullname,data,'-append','precision',8); %With this statement, the record file will be created automatically if it doesn't exist.
        end
    end
end