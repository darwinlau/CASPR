classdef ArduinoCASPRInterface < handle
    properties (Constant)
        M_TO_MM = 1000;
        MM_TO_M = 0.001;
        LENGTH_HEX_NUM_DIGITS = 4;
    end
    
    properties (SetAccess = private)
        numCmd              % Number of actuator command values to send and feedback
        comPort             % Serial COM port connected to Arduino board
        % Length multiplier refers to value to keep the data as an integer.
        % For example if multiplier = 10, then 4.8 (mm) becomes 48 to be
        % sent to the hardware, and the vice versa, 48 received refers to
        % 4.8 mm to increase the resolution of the data
        lengthMult = 10;
    end
    
    properties (Access = private)
        serial_port
    end
    
    properties (Dependent)
    end
    
    properties (Constant)
        BAUD_RATE = 115200;     % Bits per second
    end
    
    methods
        function interface = ArduinoCASPRInterface(comPort, numCmd)
            interface.comPort = comPort;
            interface.numCmd = numCmd;
            interface.setupSerial(interface.comPort);
        end
                
        function close(obj)
            %clear all;
            if ~isempty(obj.serial_port)
                fclose(obj.serial_port);
                delete(obj.serial_port);
                obj.serial_port = [];
            end
            %close all
            disp('Serial Port Closed');
        end
        
        % Sends character 'a' to the device and waits for same character to
        % come back
        function [success] = dectectDevice(obj)
            fprintf(obj.serial_port, 'a\n');
            %set(obj.serial_port, 'Timeout', 5.0);
            cmd_str = fscanf(obj.serial_port, '%s\n');
            if (isequal(cmd_str, 'a'))
                success = true;
            else
                success = false;
            end
        end
        
        % All length commands are to be sent to hardware in terms of [mm]
        % Note: l_cmd from CASPR will be in the units [m]
        function lengthCommandSend(obj, l_cmd)
            CASPR_log.Assert(length(l_cmd) == obj.numCmd, sprintf('Number of command values must be equal to %d', obj.numCmd));
            l_cmd_units = obj.M_TO_MM * obj.lengthMult * l_cmd;
            str_cmd = 'L';
            for i = 1:obj.numCmd
                str_cmd = [str_cmd, dec2hex(round(l_cmd_units(i)), obj.LENGTH_HEX_NUM_DIGITS)];
            end
            fprintf(obj.serial_port, '%s\n', str_cmd);
        end
        
        % All length commands are to be sent to hardware in terms of [mm]
        % Note: l_cmd from CASPR will be in the units [m]
        function lengthInitialSend(obj, l0)
            CASPR_log.Assert(length(l0) == obj.numCmd, sprintf('Number of command values must be equal to %d', obj.numCmd));
            l0_units = obj.M_TO_MM * obj.lengthMult*l0;
            str_cmd = 'I';
            for i = 1:obj.numCmd
                str_cmd = [str_cmd, dec2hex(l0_units(i), obj.LENGTH_HEX_NUM_DIGITS)];
            end
            fprintf(obj.serial_port, '%s\n', str_cmd);
        end
        
        % All length commands are to sent from hardware in terms of [mm]
        % Note: l_feedback from CASPR will be in the units [m]
        function [l_feedback] = feedbackRead(obj)
            cmd_str = fscanf(obj.serial_port, '%s\n');
            CASPR_log.Assert(length(cmd_str) == obj.LENGTH_HEX_NUM_DIGITS * obj.numCmd + 1, sprintf('Number of feedback values must be equal to %d', obj.numCmd));
            CASPR_log.Assert(cmd_str(1) == 'F', 'First character of feedback should be ''F''');
            l_feedback = zeros(obj.numCmd, 1);
            % Start at character 2 since 1st character should be 'F'
            cmd_str_ind = 2;
            for i = 1:obj.numCmd
                l_feedback(i) = hex2dec(cmd_str(cmd_str_ind:cmd_str_ind+obj.LENGTH_HEX_NUM_DIGITS-1));
                cmd_str_ind = cmd_str_ind + obj.LENGTH_HEX_NUM_DIGITS;                
            end
            l_feedback = l_feedback * obj.MM_TO_M / obj.lengthMult;
            l_feedback
        end
        
        function systemOnSend(obj)
            fprintf(obj.serial_port, 's\n');
        end
        
        function systemOffSend(obj)
            fprintf(obj.serial_port, 'e\n');
        end
    end
    
    
    methods (Access = private)
        function setupSerial(obj, comPort)
            % Initialize the serialpush port communication between Arduino and MATLAB
            % The input value is the COMPORT should be changed as per requirement
            % We ensure that the arduino is also communicatiing with MATLAB at this
            % time. A predefined code on the arduino acknowledges this. 

            obj.serial_port = serial(comPort);
            set(obj.serial_port, 'DataBits', 8);
            set(obj.serial_port, 'StopBits', 1);
            set(obj.serial_port, 'BaudRate', obj.BAUD_RATE);
            set(obj.serial_port, 'Parity', 'none'); 
            
            try
                fopen(obj.serial_port);
            catch error
                CASPR_log.Error(error.message);
            end
        end
        
    end
    
    methods (Static)
        function CloseSerialAll()
            if ~isempty(instrfind)
                fclose(instrfind);
                delete(instrfind);
            end
        end
    end
end

