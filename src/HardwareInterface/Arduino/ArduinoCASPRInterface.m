classdef ArduinoCASPRInterface < HardwareInterfaceBase
    properties (Constant)
        M_TO_MM = 1000;
        MM_TO_M = 0.001;
        LENGTH_HEX_NUM_DIGITS = 4;
        RECEIVE_PREFIX_FEEDBACK = 'f';
        RECEIVE_PREFIX_ERROR = 'e';
        SEND_PREFIX_START = 's';
        SEND_PREFIX_END = 'e';
        SEND_PREFIX_INITIAL = 'i';
        SEND_PREFIX_LENGTH_CMD = 'l';
        COMM_PREFIX_ACKNOWLEDGE = 'a';
        COMM_PREFIX_SETUP = 'k';
        
        BAUD_RATE = 74880;     % Bits per second
    end
    
    properties (SetAccess = private)
        numCmd              % Number of actuator command values to send and feedback
        comPort             % Serial COM port connected to Arduino board
        % Length multiplier refers to value to keep the data as an integer.
        % For example if multiplier = 10, then 4.8 (mm) becomes 48 to be
        % sent to the hardware, and the vice versa, 48 received refers to
        % 4.8 mm to increase the resolution of the data
        lengthMult = 10;
        
        feedback
    end
    
    properties (Access = private)
        serial_port
    end
    
    methods
        function interface = ArduinoCASPRInterface(comPort, numCmd)
            interface@HardwareInterfaceBase();
            interface.comPort = comPort;
            interface.numCmd = numCmd;
        end
        
        function open(obj)
            % Initialize the serialpush port communication between Arduino and MATLAB
            % The input value is the COMPORT should be changed as per requirement
            % We ensure that the arduino is also communicatiing with MATLAB at this
            % time. A predefined code on the arduino acknowledges this.
            
            obj.serial_port = serial(obj.comPort);
            set(obj.serial_port, 'DataBits', 8);
            set(obj.serial_port, 'StopBits', 1);
            set(obj.serial_port, 'BaudRate', obj.BAUD_RATE);
            set(obj.serial_port, 'Parity', 'none');
            
            try
                fopen(obj.serial_port);
            catch error
                CASPR_log.Error(error.message);
            end
            
            pause(1.0);
            fprintf(obj.serial_port, [obj.COMM_PREFIX_SETUP '\n']);
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
        
        % Sends character to the device and waits for same character to
        % come back
        function [success] = detectDevice(obj)
            fprintf(obj.serial_port, [obj.COMM_PREFIX_ACKNOWLEDGE '\n']);
            cmd_str = fscanf(obj.serial_port, '%s\n');
            if (isequal(cmd_str, obj.COMM_PREFIX_ACKNOWLEDGE))
                success = true;
            else
                success = false;
            end
        end
        
        % All length commands are to be sent to hardware in terms of [mm]
        % Note: l_cmd from CASPR will be in the units [m]
        function lengthCommandSend(obj, l_cmd)
            %l_cmd = l_cmd(1:1);
            CASPR_log.Assert(length(l_cmd) == obj.numCmd, sprintf('Number of command values must be equal to %d', obj.numCmd));
            str_cmd = obj.SEND_PREFIX_LENGTH_CMD;
            for i = 1:obj.numCmd
                str_cmd = [str_cmd, obj.casprLengthToHW(l_cmd(i))];
            end
            fprintf(obj.serial_port, '%s\n', str_cmd);
        end
        
        % All length commands are to be sent to hardware in terms of [mm]
        % Note: l_cmd from CASPR will be in the units [m]
        function lengthInitialSend(obj, l0)
            CASPR_log.Assert(length(l0) == obj.numCmd, sprintf('Number of command values must be equal to %d', obj.numCmd));
            str_cmd = obj.SEND_PREFIX_INITIAL;
            for i = 1:obj.numCmd
                str_cmd = [str_cmd, obj.casprLengthToHW(l0(i))];
            end
            fprintf(obj.serial_port, '%s\n', str_cmd);
        end
        
        function cmdRead(obj)
            cmd_str = fscanf(obj.serial_port, '%s\n');
            %            if (cmd_str(1) == obj.RECEIVE_PREFIX_FEEDBACK)
            %obj.feedbackRead(cmd_str);
            %            elseif (cmd_str(1) == obj.RECEIVE_PREFIX_ERROR && length(cmd_str) == 1)
            %               CASPR_log.Error('Error received from Arduino hardware interface');
            %  else
            cmd_str
            %            end
        end
        
        % All length commands are to sent from hardware in terms of [mm]
        % Note: l_feedback from CASPR will be in the units [m]
        function feedbackRead(obj, cmd_str)
            CASPR_log.Assert(length(cmd_str) == obj.LENGTH_HEX_NUM_DIGITS * obj.numCmd + 1, sprintf('Number of feedback values must be equal to %d', obj.numCmd));
            CASPR_log.Assert(cmd_str(1) == obj.RECEIVE_PREFIX_FEEDBACK, 'First character of feedback should be ''F''');
            obj.feedback = zeros(obj.numCmd, 1);
            % Start at character 2 since 1st character should be 'f'
            cmd_str_ind = 2;
            for i = 1:obj.numCmd
                obj.feedback(i) = obj.hardwareLengthToCASPR(cmd_str(cmd_str_ind:cmd_str_ind+obj.LENGTH_HEX_NUM_DIGITS-1));
                cmd_str_ind = cmd_str_ind + obj.LENGTH_HEX_NUM_DIGITS;
            end
        end
        
        function systemOnSend(obj)
            fprintf(obj.serial_port, [obj.SEND_PREFIX_START '\n']);
        end
        
        function systemOffSend(obj)
            fprintf(obj.serial_port, [obj.SEND_PREFIX_END '\n']);
        end
    end
    
    methods (Access = private)
        % Unit on hardware [mm] * lengthMult (to increase resolution) sent to CASPR in hex format
        function caspr_length = hardwareLengthToCASPR(obj, hardware_length_str)
            caspr_length = hex2dec(hardware_length_str) * obj.MM_TO_M / obj.lengthMult;
        end
        
        % Unit on CASPR [m], convert to [mm] with multiplier and in HEX to
        % a particular number of digits
        function hw_length_str = casprLengthToHW(obj, length)
            hw_length_str = dec2hex(round(length * obj.M_TO_MM * obj.lengthMult), obj.LENGTH_HEX_NUM_DIGITS);
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

