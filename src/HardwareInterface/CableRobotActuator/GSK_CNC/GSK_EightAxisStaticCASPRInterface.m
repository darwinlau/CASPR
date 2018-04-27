classdef GSK_EightAxisStaticCASPRInterface < CableActuatorInterfaceBase
    properties (Constant)
        CABLE_PREFIX_ID = {'X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V'};
        CABLE_SPEED_PREFIX = 'F';
        MAX_CABLE_NUM = 8;
        M_PER_S_TO_MM_PER_MIN = 1000*60;
        
        S_TO_MS = 1000;
        M_TO_MM = 1000;
        MIN_TO_S = 60;
        MM_DECIMAL_PLACE = -3;
                       
        CABLE_LENGTH_OFFSET_MM = [1.1293; 0.8382; 0.9429; 1.0143; 1.1150; 2.0500; 1.2591; 1.1561]; % NOTE: UNITS IN MM
        
        MACHINE_LOW_F_SPEED = 5000;
    end
    
    properties 
        filename
        filefolder
        timeStep
    end 
    
    properties (Access = private)        
        fileHandle
        modelObj
        
        l_zero_mm
        l_prev_mm
    end
        
    methods
        % Note: l_init from CASPR will be in the units [m]
        function interface = GSK_EightAxisStaticCASPRInterface(modelObj, filefolder, filename, zero_cable_lengths_m)
            CASPR_log.Assert(modelObj.numCables <= GSK_EightAxisStaticCASPRInterface.MAX_CABLE_NUM, sprintf('Number of cables must less than or equal to %d', GSK_EightAxisStaticCASPRInterface.MAX_CABLE_NUM));
            interface@CableActuatorInterfaceBase();
            interface.modelObj = modelObj;
            interface.filefolder = filefolder;
            interface.filename = filename;
            interface.l_zero_mm = zero_cable_lengths_m * interface.M_TO_MM;
            %interface.timeStep = time_step;
        end
        
        function open(obj)
            obj.fileHandle = fopen([obj.filefolder obj.filename '0.txt'], 'w');
            fprintf(obj.fileHandle, '%s0;\r\n', obj.filename);
        end
                
        function close(obj)
            fclose(obj.fileHandle);
        end
        
        % Always true for this interface as there is no physical device
        function [success] = detectDevice(~)
            success = 1;
        end
        
        function forceFeedbackRead(~)
        end
        
        function forceCommandSend(~,~)
        end
        
        function switchOperatingMode(~,~)
        end
        
        function initialise(~)
        end
        
        % All length commands are to be sent to hardware in terms of [mm]
        % Note: l_cmd from CASPR will be in the units [m]
        function lengthCommandSend(obj, l_cmd)
            l_cmd_mm = l_cmd * obj.M_TO_MM;
            l_rel_mm = l_cmd_mm - obj.CABLE_LENGTH_OFFSET_MM;
            
            % First compute the F speed
            speed_mm_per_s = norm(l_rel_mm - obj.l_prev_mm)/obj.timeStep;
            speed_mm_per_min = speed_mm_per_s * obj.MIN_TO_S;
            F_val = round(speed_mm_per_min);
            
            if (F_val == 0 || all(abs(l_rel_mm - obj.l_prev_mm) < 10^(obj.MM_DECIMAL_PLACE)))
                fprintf(obj.fileHandle, 'G04 P%d;\r\n', round(obj.timeStep * obj.S_TO_MS));
            else
                % G90 for absolute machine coordinates
                % G91 for relative machine coordinates
                fprintf(obj.fileHandle, 'G01 G90');
                l_print = roundn(obj.toAbsoluteMachineLength(l_rel_mm), obj.MM_DECIMAL_PLACE);
                for i = 1:obj.modelObj.numCables
                    CASPR_log.Assert(ismember(obj.modelObj.cableModel.cables{i}.name, obj.CABLE_PREFIX_ID), 'Cable name must be one of the prefices ''XYZABCUV''');
                    fprintf(obj.fileHandle, ' %c%.3f', obj.modelObj.cableModel.cables{i}.name, l_print(i));
                end
                fprintf(obj.fileHandle, ' %c%d;\r\n', obj.CABLE_SPEED_PREFIX, F_val);
                obj.l_prev_mm = l_rel_mm;
            end
        end
        
        % Make the system go to the initial position as set using G00 mode
        % with the offset
        function lengthInitialSend(obj, l0)
            % Initial length at full precision
            l0_mm = l0 * obj.M_TO_MM;
            
            % Store the initial length with offset as the "l_prev_mm" for
            % computing if a pause is needed during trajectory
            obj.l_prev_mm =  l0_mm - obj.CABLE_LENGTH_OFFSET_MM;
            
            % First write to the motion file 'XXXXX0'
            fprintf(obj.fileHandle, 'G00');
            l_rel_mm = roundn(obj.toAbsoluteMachineLength(l0_mm) - obj.CABLE_LENGTH_OFFSET_MM, obj.MM_DECIMAL_PLACE);
            for i = 1:obj.modelObj.numCables
                % Write the initial length WITH OFFSET rounded to right DP
                fprintf(obj.fileHandle, ' %c%.3f', obj.modelObj.cableModel.cables{i}.name, l_rel_mm(i));
            end
            fprintf(obj.fileHandle, ';\r\n');
                        
            % Print the initial lengths program (no length offset) to 'XXXXX9' file          
            fp = fopen([obj.filefolder obj.filename '9.txt'], 'w');
            fprintf(fp, '%s9;\r\n', obj.filename);
            fprintf(fp, 'G01 G90');
            l_rel_mm = roundn(obj.toAbsoluteMachineLength(l0_mm), obj.MM_DECIMAL_PLACE);
            for i = 1:obj.modelObj.numCables
                fprintf(fp, ' %c%.3f', obj.modelObj.cableModel.cables{i}.name, l_rel_mm(i));
            end
            fprintf(fp, ' F%d;\r\n', obj.MACHINE_LOW_F_SPEED);
            fprintf(fp, 'M30;\r\n');
            fprintf(fp, '%%\r\n');
            fclose(fp);
            
            % Print the initial lengths program (with length offset) to 'XXXXX8' file  
            fp = fopen([obj.filefolder obj.filename '8.txt'], 'w');
            fprintf(fp, '%s8;\r\n', obj.filename);
            fprintf(fp, 'G01 G90');
            l_rel_mm = roundn(obj.toAbsoluteMachineLength(l0_mm) - obj.CABLE_LENGTH_OFFSET_MM, obj.MM_DECIMAL_PLACE);
            for i = 1:obj.modelObj.numCables
                fprintf(fp, ' %c%.3f', obj.modelObj.cableModel.cables{i}.name, l_rel_mm(i));
            end
            fprintf(fp, ' F%d;\r\n', obj.MACHINE_LOW_F_SPEED);
            fprintf(fp, 'M30;\r\n');
            fprintf(fp, '%%\r\n');
            fclose(fp);
        end
        
        % Do nothing since there's no meaning for this interface
        function cmdRead(~)
        end
        
        % Do nothing since there's no meaning for this interface (yet)
        function lengthFeedbackRead(~, ~)
        end
        
        % Do nothing since there's no meaning for this interface
        function systemOnSend(~)            
        end
        
        % Print in the closing commands for the script
        function systemOffSend(obj)
            fprintf(obj.fileHandle, 'M30;\r\n');
            fprintf(obj.fileHandle, '%%\r\n');
        end
        
        % Converts the length (absolute cable length) to the absolute
        % machine length (where zero is l_zero_mm)
        function rel_l_mm = toAbsoluteMachineLength(obj, abs_l_mm)
            rel_l_mm = zeros(obj.modelObj.numCables, 1);
            for i = 1:obj.modelObj.numCables
                rel_l_mm(i,1) = abs_l_mm(i) - obj.l_zero_mm(find([obj.CABLE_PREFIX_ID{:}] == obj.modelObj.cableModel.cables{i}.name));
            end
            %rel_l_mm = abs_l_mm - obj.l_zero_mm;
        end
    end
end

