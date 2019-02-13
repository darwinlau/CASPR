classdef Laser < handle
    properties (Access = public)
        laser
        strComPort
        comPort
        send_list
        currentIntensity = 0;

        us_enable = 0;
        us_distance
        us_timer = 0;
    end
    
    properties (Access = public, Constant = true)
        BAUDRATE       = 9600;
              
        CMD_MOTION_WRITE_HEADER = [hex2dec('FF') hex2dec('AA')];
        CMD_ULTRASONIC_READ = uint8(hex2dec(['FF';'01';'FF';'FF']))';
        SERIAL_TIMEOUT = 1;%s
        
        US_BYTES_NUM = 1;
        US_READ_PERIOD = 1;%s
        
        % The default value is 1 second. The minimum value is 0.01 second.
        TIMERPERIOD = 0.05;%s
        
        OUTPUTBUFFERSIZE = 4;
    end
    
    methods (Access = public)
        
        function laser = Laser(strComPort)
            laser.strComPort = strComPort;
            import java.util.LinkedList
            laser.send_list = LinkedList();
        end
        
        function initialize(obj)
            obj.comPort = serial(obj.strComPort);
            obj.comPort.BaudRate = obj.BAUDRATE;
            obj.comPort.timeout = obj.SERIAL_TIMEOUT;
            obj.comPort.ReadAsyncMode = 'continuous';
            
            obj.comPort.TimerPeriod = obj.TIMERPERIOD; % s
            obj.comPort.TimerFcn = {@serial_callback,obj};
            
            obj.openserial();
        end
        
        function disconnect(obj)
            obj.closeserial();
            delete(instrfind('Name',strcat('serial-',obj.strComPort)));
        end
        
        function setLaser(obj, laser)
           obj.laser = laser;
           if 1 %obj.laser ~= obj.currentIntensity
               sendLaser(obj);
               obj.currentIntensity = obj.laser;
           end
        end
        
        function sendLaser(obj)
            cmd = [obj.laser];
            fprintf(obj.comPort, num2str(cmd))
            obj.currentIntensity = cmd;
        end
        
        function laserOff(obj)
            fprintf(obj.comPort, '0');
        end
        
        function update_us_state(obj)
            obj.us_distance = fread(obj.comPort,obj.US_BYTES_NUM,'int8');
        end
        
        function setUS_ENABLE(obj, isEnable)
            if(~isEnable)
                obj.us_distance = [];
            end
            obj.us_enable = isEnable;
        end
        function timer_callback(obj)
%             if(obj.us_enable)
%                 if(obj.us_timer == 0)
%                     % upload the us cmd
%                     obj.send_list.add(double(obj.CMD_ULTRASONIC_READ));
%                 end
%                 obj.us_timer = obj.us_timer + 1;
%                 if(obj.us_timer >= obj.US_READ_PERIOD/obj.TIMERPERIOD)
%                     obj.us_timer = 0;
%                 end
%             else
%                 obj.us_timer = 0;
%             end
            
            if(obj.send_list.size() > 0 && ...
                (strcmpi(obj.comPort.TransferStatus,'read') || strcmpi(obj.comPort.TransferStatus,'idle')))
                obj.sendCMD(obj.send_list.remove());
            end
          end
         function us_distance = getUltraSonic(obj)
            us_distance = obj.us_distance;
       end 
    end
    
    methods (Access = public)
        function openserial(obj)
            fopen(obj.comPort);
        end
        
        function closeserial(obj)
            stopasync(obj.comPort);
            pause(1);
            fclose(obj.comPort);
            delete(obj.comPort);
            clear obj.comPort;
        end
        
        function sendReadCMD(obj)
            obj.send_list.add(obj.CMD_ULTRASONIC_READ);
        end
        
        function sendCMD(obj, cmd)
            if(strcmpi(obj.comPort.TransferStatus,'read') || strcmpi(obj.comPort.TransferStatus,'idle'))
                cmd_intermedia = uint8(cmd);
                fwrite(obj.comPort,cmd_intermedia,'async');
            end
        end
    end
end

function serial_callback(obj, event, laser)
% Determine the type of event.
EventType = event.Type;

% if strcmpi(EventType, 'BytesAvailable')
%     laser.update_us_state();
% end

if strcmpi(EventType, 'Timer')
    %     laser.trigger_us();
    laser.timer_callback();
end

% if strcmpi(EventType, 'OutputEmpty')
%     laser.update_write_state();
% end
end

