classdef Laser < handle
    properties (Access = public)
        hand_angle
        arm_angle
        laser %for laser project
        strComPort
        comPort
        send_list

        us_enable = 0;
        us_distance
        us_timer = 0;
    end
    
    properties (Access = public, Constant = true)
        BAUDRATE       = 9600;
        
        MAX_HAND_ANGLE = 180;
        MIN_HAND_ANGLE = 70;
        INI_HAND_ANGLE = 180;
        
        BEST_HAND_ANGLE = 103;% For gripping
        RELEASE_HAND_ANGLE = 125; % For releasing the brick
        LOOSE_HAND_ANGLE = 146;
        
        
        MAX_ARM_ANGLE  = 180;
        MIN_ARM_ANGLE  = 0;
        INI_ARM_ANGLE  = 90;
        
        CMD_MOTION_WRITE_HEADER = [hex2dec('FF') hex2dec('AA')];
        CMD_ULTRASONIC_READ = uint8(hex2dec(['FF';'01';'FF';'FF']))';
        SERIAL_TIMEOUT = 1;%s
        
        US_BYTES_NUM = 1;
        US_READ_PERIOD = 1;%s
        
        % The default value is 1 second. The minimum value is 0.01 second.
        TIMERPERIOD = 0.05;%s
    end
    
    methods (Access = public)
        
        function gripper = Gripper(strComPort)
            gripper.strComPort = strComPort;
            gripper.arm_angle  = gripper.INI_ARM_ANGLE;
            import java.util.LinkedList
            gripper.send_list = LinkedList();
        end
        
        function initialize(obj)
            obj.comPort = serial(obj.strComPort);
            obj.comPort.BaudRate = obj.BAUDRATE;
            obj.comPort.timeout = obj.SERIAL_TIMEOUT;
            obj.comPort.ReadAsyncMode = 'continuous';
            
            % async read settings
%             obj.comPort.BytesAvailableFcnCount = obj.US_BYTES_NUM;
%             obj.comPort.BytesAvailableFcnMode = 'byte';
%             obj.comPort.BytesAvailableFcn = {@serial_callback,obj};
            
            obj.comPort.TimerPeriod = obj.TIMERPERIOD; % s
            obj.comPort.TimerFcn = {@serial_callback,obj};
            
%             obj.comPort.OutputEmptyFcn = {@serial_callback,obj};
            
            obj.openserial();
        end
        
        function disconnect(obj)
            obj.closeserial();
        end
        
        function setHandAngle(obj,hand_angle)
%             CASPR_log.Assert(hand_angle<= obj.MAX_HAND_ANGLE && hand_angle>=obj.MIN_HAND_ANGLE,'The argument is out of range!');
            obj.hand_angle = hand_angle;
            cmd = [obj.CMD_MOTION_WRITE_HEADER obj.hand_angle obj.arm_angle];
            obj.send_list.add(cmd);
            obj.send_list.add(cmd);
        end
        
        %for laser project
        function setLaser(obj,laser)
%             CASPR_log.Assert(hand_angle<= obj.MAX_HAND_ANGLE && hand_angle>=obj.MIN_HAND_ANGLE,'The argument is out of range!');
            obj.laser = laser;
            cmd = [obj.laser];
            obj.send_list.add(cmd);
            obj.send_list.add(cmd);
        end
        
        function setArmAngle(obj,arm_angle)
            CASPR_log.Assert(arm_angle<= obj.MAX_ARM_ANGLE && arm_angle>=obj.MIN_ARM_ANGLE,'The argument is out of range!');
            % calibration of gripper B
            b_ang = 9; c_ang = 99; d_ang = 180;
            a_ang = c_ang - (92/90)*(c_ang-b_ang);
            
            if (arm_angle < 92)
                temp = (1/92)*(c_ang-a_ang)*(92-arm_angle)+a_ang;                
            else
                temp = ((180-arm_angle)*(d_ang-c_ang))/(180-92)+c_ang;                
            end
%             % calibration variables
%             angle180 = 90;% 90 DEGREE real;
%             angle0 = 274;%DEGREE
%             k = (angle180-angle0)/(180-0);
%             if(arm_angle>=90)
%                 temp = (arm_angle-angle0)/k;
%             else
%                 temp = (arm_angle-angle180)/k;
%             end
            % model easily: equation of one unknow, one order
%             angle0 = 187; % decimalism
%             angle180 = -3;
%             k = (angle180-angle0)/(180-0);
%             temp = (arm_angle-angle0)/k;
%             if temp<0
%                 temp = temp + 180;
%             elseif temp>180
%                 temp = temp - 180;
%             end
            
            obj.arm_angle = temp;
            cmd = [obj.CMD_MOTION_WRITE_HEADER obj.hand_angle obj.arm_angle];
            obj.send_list.add(cmd);
            obj.send_list.add(cmd);
        end
        
%         function setHandArm_Angle(obj,hand_angle,arm_angle)
%             CASPR_log.Assert(arm_angle<= obj.MAX_ARM_ANGLE && arm_angle>=obj.MIN_ARM_ANGLE,'The argument is out of range!');
%             CASPR_log.Assert(hand_angle<= obj.MAX_HAND_ANGLE && hand_angle>=obj.MIN_HAND_ANGLE,'The argument is out of range');
%             obj.hand_angle = hand_angle;
%             obj.arm_angle = arm_angle;
%             cmd = [obj.CMD_MOTION_WRITE_HEADER obj.hand_angle obj.arm_angle];
%             obj.send_list.add(cmd);
%             obj.send_list.add(cmd);
%             obj.send_list.add(cmd);
%         end

        function us_distance = getUltraSonic(obj)
            us_distance = obj.us_distance;
        end
        
        
%         us_enable = 0;
%         US_BYTES_NUM = 1;
%         US_READ_PERIOD = 1;%s
%         
%         % The default value is 1 second. The minimum value is 0.01 second.
%         TIMERPERIOD = 0.05;%s
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
        
%         function trigger_us(obj)
%             flushinput(obj.comPort);
%             obj.sendReadCMD();
%             %readasync(obj.comPort, obj.US_BYTES_NUM);
%         end
        
%         function update_write_state(obj)
%             if(obj.send_list.size() > 0 && ...
%                 (strcmpi(obj.comPort.TransferStatus,'read') || strcmpi(obj.comPort.TransferStatus,'idle')))
%                 obj.sendCMD(obj.send_list.remove());
%             end
%         end
        
        function update_us_state(obj)
            obj.us_distance = fread(obj.comPort,obj.US_BYTES_NUM,'int8');
        end
        
        function setUS_ENABLE(obj, isEnable)
            if(~isEnable)
                obj.us_distance = [];
            end
            obj.us_enable = isEnable;
        end
    end
    
    methods (Access = private)
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

function serial_callback(obj, event, gripper)
% Determine the type of event.
EventType = event.Type;

% if strcmpi(EventType, 'BytesAvailable')
%     gripper.update_us_state();
% end

if strcmpi(EventType, 'Timer')
    %     gripper.trigger_us();
    gripper.timer_callback();
end

% if strcmpi(EventType, 'OutputEmpty')
%     gripper.update_write_state();
% end
end

