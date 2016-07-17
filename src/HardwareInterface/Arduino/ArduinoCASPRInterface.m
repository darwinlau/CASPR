classdef ArduinoCASPRInterface < handle
    %ARDUINOCASPR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        comPort
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
        function interface = ArduinoCASPRInterface(comPort)
            interface.comPort = comPort;
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
        
        function lengthCommandSend(obj)
        end
        
        function lengthInitialSend(obj)
        end
        
        function feedbackRead(obj)
            cmd_str = fscanf(obj.serial_port, '%s\n');
            disp(cmd_str);
        end
        
        function feedbackOnSend(obj)
            fprintf(obj.serial_port, 's\n');
        end
        
        function feedbackOffSend(obj)
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

