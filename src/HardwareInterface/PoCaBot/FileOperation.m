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
    
    methods
        function fo = FileOperation(filepath_record, filepath_pickup_co, filepath_place_co)
            fo.filepath_record = filepath_record;
            fo.filepath_pickup_co = filepath_pickup_co;
            fo.filepath_place_co = filepath_place_co;
            
            fo.readCoFile();
        end
        
        function [pickup_co, place_co]= getCoordinate(obj, num)
            pickup_co = obj.pickup_list(1:3,num);
            place_co = obj.place_list(1:3,num);
        end
        
        function [arm_angle_pickup, arm_angle_place] = getArmAngle(obj,num)
            arm_angle_pickup = obj.pickup_list(4,num);
            arm_angle_place = obj.place_list(4,num);
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
            num = size(obj.pickup_list,2);
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
            %
            %             fileID = fopen(obj.filepath_place_co,'r');
            %             formatSpec = '%f %f %f %f';
            %             size_place_list = [4 Inf];
            %             obj.place_list = fscanf(fileID, formatSpec, size_place_list);
            %             fclose(fileID);
            obj.place_list = load(obj.filepath_place_co);
            
            CASPR_log.Assert(size(obj.pickup_list) == size(obj.place_list), ...
                'The coordinate files of the picking up and placing do not match each other!');
        end
    end
end