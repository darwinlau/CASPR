classdef GSK_EightAxisStaticCASPRInterface < HardwareInterfaceBase
    properties (Constant)
        CABLE_PREFIX_ID = {'X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V'};
        CABLE_SPEED_PREFIX = 'F';
        MAX_CABLE_NUM = 8;
        
        S_TO_MS = 1000;
        M_TO_MM = 1000;
        MIN_TO_S = 60;
        MM_DECIMAL_PLACE = -3;
        
        
        M_PER_S_TO_MM_PER_MIN = 1000*60;
    end
    
    properties 
        filename
        filefolder
        timeStep
    end 
    
    properties (Access = private)        
        fileHandle
        numCmd
        
        l_init_mm
        l_prev_mm
    end
        
    methods
        function interface = GSK_EightAxisStaticCASPRInterface(numCmd, filefolder, filename)
            CASPR_log.Assert(numCmd <= GSK_EightAxisStaticCASPRInterface.MAX_CABLE_NUM, sprintf('Number of cables must less than or equal to %d', GSK_EightAxisStaticCASPRInterface.MAX_CABLE_NUM));
            interface@HardwareInterfaceBase();
            interface.numCmd = numCmd;
            interface.filefolder = filefolder;
            interface.filename = filename;
            %interface.timeStep = time_step;
        end
        
        function open(obj)
            obj.fileHandle = fopen([obj.filefolder obj.filename '.txt'], 'w');
        end
                
        function close(obj)
            fclose(obj.fileHandle);
        end
        
        % Always true for this interface as there is no physical device
        function [success] = detectDevice(~)
            success = 1;
        end
        
        % All length commands are to be sent to hardware in terms of [mm]
        % Note: l_cmd from CASPR will be in the units [m]
        function lengthCommandSend(obj, l_cmd)
            l_cmd_mm = roundn(l_cmd * obj.M_TO_MM, obj.MM_DECIMAL_PLACE);
            
            if (l_cmd_mm == obj.l_prev_mm)
                fprintf(obj.fileHandle, 'G04 P%d;\r\n', round(obj.timeStep * obj.S_TO_MS));
            else
                l_rel_mm = l_cmd_mm - obj.l_init_mm;
                fprintf(obj.fileHandle, 'G01');
                for i = 1:obj.numCmd
                    fprintf(obj.fileHandle, ' %c%.3f', obj.CABLE_PREFIX_ID{i}, l_rel_mm(i));
                end
                speed_mm_per_s = norm(l_cmd_mm - obj.l_prev_mm)/obj.timeStep;
                speed_mm_per_min = speed_mm_per_s * obj.MIN_TO_S;
                F_val = round(speed_mm_per_min);
                fprintf(obj.fileHandle, ' %c%d;\r\n', obj.CABLE_SPEED_PREFIX, F_val);
            end
            
            obj.l_prev_mm = l_cmd_mm;
        end
        
        % Do nothing since there's no meaning for this interface
        function lengthInitialSend(obj, l0)
            obj.l_init_mm = roundn(l0 * obj.M_TO_MM, obj.MM_DECIMAL_PLACE);
            obj.l_prev_mm = roundn(l0 * obj.M_TO_MM, obj.MM_DECIMAL_PLACE);
        end
        
        % Do nothing since there's no meaning for this interface
        function cmdRead(~)
        end
        
        % Do nothing since there's no meaning for this interface (yet)
        function feedbackRead(~, ~)
        end
        
        % Do nothing since there's no meaning for this interface
        function systemOnSend(obj)            
            fprintf(obj.fileHandle, '%s;\r\n', obj.filename);
        end
        
        % Do nothing since there's no meaning for this interface
        function systemOffSend(obj)
            fprintf(obj.fileHandle, 'M30;\r\n');
            fprintf(obj.fileHandle, '%%\r\n');
        end
    end
end

