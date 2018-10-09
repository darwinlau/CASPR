classdef Gripper < handle
    properties (Access = public)
        hand_angle_cmd = Gripper.LOOSE_HAND_ANGLE;
        arm_angle_cmd = 90;
        arm_angle_needed
        
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
        
        BEST_HAND_ANGLE = hex2dec('52');% For gripping
        RELEASE_HAND_ANGLE = hex2dec('5F'); % For releasing the brick
        LOOSE_HAND_ANGLE = hex2dec('6E');% 90 70
        
        
        MAX_ARM_ANGLE  = 180;
        MIN_ARM_ANGLE  = 0;
        INI_ARM_ANGLE  = 90;
        
        LOOKUP_ARM_ANGLE = [  ...
            hex2dec('00'), 106;...
            hex2dec('10'), 90;...
            hex2dec('37'), 50;...
            hex2dec('61'),  7;...
            hex2dec('68'),  0;...
            hex2dec('8A'),-32;...
            hex2dec('B4'),-80];
%         LOOKUP_ARM_ANGLE = [  ...
%             hex2dec('08'), 185;...
%             hex2dec('0E'), 180;...
%             hex2dec('37'), 135;...
%             hex2dec('61'),  90;...
%             hex2dec('8A'),  50;...
%             hex2dec('B4'),   5];
        
%         LOOKUP_DISTANCE_SENSOR1 = [  ...
%             49.7000   -0.0250
%             47.2600   -0.0100
%             40.2500         0
%             35.9800    0.0100
%             31.2400    0.0200
%             28.2400    0.0300];
%         
%         LOOKUP_DISTANCE_SENSOR2 = [  ...
%             55.9700   -0.0250
%             43.0100   -0.0100
%             37.0000         0
%             32.0300    0.0100
%             28.0000    0.0200
%             25.0000    0.0300];
        
        LOOKUP_DISTANCE_SENSOR1 = [  ...
            62.0100   -0.0250
            49.0200   -0.0100
            43.0200         0
            37.0000    0.0100
            32.7000    0.0200
            28.9200    0.0300];
        
        LOOKUP_DISTANCE_SENSOR2 = [  ...
            60.0000   -0.0250
            47.6900   -0.0100
            41.0000         0
            35.0600    0.0100
            31.0000    0.0200
            27.1100    0.0300];
        
        CMD_MOTOR_HEADER = [hex2dec('FF') hex2dec('AA')];
        CMD_DISTANCE_READ = uint8(hex2dec(['FF';'22';'22';'22']))';
        RET_DISTANCE_HEADER = hex2dec(['FF';'22']);
        CMD_DISABLE_POWER = uint8(hex2dec(['FF';'77';'77';'00']))';
        CMD_ENABLE_POWER = uint8(hex2dec(['FF';'77';'77';'01']))';
        
        SERIAL_TIMEOUT = 2;%s Waiting time to complete a read or write operation
        
        US_BYTES_NUM = 1;
        US_READ_PERIOD = 1;%s
        
        % The default value is 1 second. The minimum value is 0.01 second.
        TIMERPERIOD = 1;%s
    end
    
    methods (Access = public)
        
        function gripper = Gripper(strComPort)
            gripper.strComPort = strComPort;
            gripper.arm_angle_cmd  = gripper.INI_ARM_ANGLE;
            %             import java.util.LinkedList
            %             gripper.send_list = LinkedList();
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
            %            obj.comPort.TimerFcn = {@serial_callback,obj};
            
            %             obj.comPort.OutputEmptyFcn = {@serial_callback,obj};
            
            obj.openserial();
        end
        
        function disconnect(obj)
            obj.closeserial();
        end
        
        function setHandAngle(obj,hand_angle_cmd)
            %             CASPR_log.Assert(hand_angle_cmd<= obj.MAX_HAND_ANGLE && hand_angle_cmd>=obj.MIN_HAND_ANGLE,'The argument is out of range!');
            obj.hand_angle_cmd = hand_angle_cmd;
            cmd = [obj.CMD_MOTOR_HEADER obj.hand_angle_cmd obj.arm_angle_cmd];
            fwrite(obj.comPort,cmd);
        end
        

        
        function setArmAngle(obj,arm_angle_needed)
            obj.arm_angle_cmd = obj.armAngleConvert(arm_angle_needed);
            cmd = [obj.CMD_MOTOR_HEADER obj.hand_angle_cmd obj.arm_angle_cmd];
            fwrite(obj.comPort,cmd);
        end
        
        function [intensity, bState] = getDistanceRawData(obj)
            flushinput(obj.comPort);
            fwrite(obj.comPort, obj.CMD_DISTANCE_READ);
            [ret, ~, msg] = fread(obj.comPort, 4);
            if(isempty(msg) && ret(1) == obj.RET_DISTANCE_HEADER(1) && ret(2) == obj.RET_DISTANCE_HEADER(2)...
                    && ret(3)>=0 && ret(4) >=0)
                bState = true;
                intensity = ret(3:4);
            else
                bState = false;
                intensity = -1;
            end
        end
        
        % when distance is -1, it means that the object is unreachable:
        % distance is more than 3cm
        % when distance is NaN, it means that the object is very close to
        % the sensor
        function [distance, bSuccess] = getDistance(obj)
            det_cnt_max = 60;
            det_cnt = 0;
            while(true)
                [intensity, bState] = obj.getDistanceRawData();
                if(bState)
                    break;
                end
                det_cnt = det_cnt + 1;
                if(det_cnt>det_cnt_max)
                    bSuccess = false;
                    distance = -1;
                    return;
                end
                pause(1);
            end
            distance = obj.distanceConvert(intensity);
            if(intensity(1)<29)
                distance(1) = -1;
            end
            if(intensity(2)<27)
                distance(2) = -1;
            end
            bSuccess = true;
        end
        
        function enablePower(obj)
            fwrite(obj.comPort,obj.CMD_ENABLE_POWER);
        end
        
        function disablePower(obj)
            fwrite(obj.comPort, obj.CMD_DISABLE_POWER);
        end
        
        function detect(obj)
            
        end
        
        
        
        
        
        
        function setHandAngleAsync(obj,hand_angle_cmd)
            %             CASPR_log.Assert(hand_angle_cmd<= obj.MAX_HAND_ANGLE && hand_angle_cmd>=obj.MIN_HAND_ANGLE,'The argument is out of range!');
            obj.hand_angle_cmd = hand_angle_cmd;
            cmd = [obj.CMD_MOTOR_HEADER obj.hand_angle_cmd obj.arm_angle_cmd];
            obj.send_list.add(cmd);
        end
        
        function setArmAngleAsync(obj,arm_angle_cmd)
            CASPR_log.Assert(arm_angle_cmd<= obj.MAX_ARM_ANGLE && arm_angle_cmd>=obj.MIN_ARM_ANGLE,'The argument is out of range!');
            % calibration of gripper B
            b_ang = 9; c_ang = 99; d_ang = 180;
            a_ang = c_ang - (92/90)*(c_ang-b_ang);
            
            if (arm_angle_cmd < 92)
                temp = (1/92)*(c_ang-a_ang)*(92-arm_angle_cmd)+a_ang;
            else
                temp = ((180-arm_angle_cmd)*(d_ang-c_ang))/(180-92)+c_ang;
            end
            %             % calibration variables
            %             angle180 = 90;% 90 DEGREE real;
            %             angle0 = 274;%DEGREE
            %             k = (angle180-angle0)/(180-0);
            %             if(arm_angle_cmd>=90)
            %                 temp = (arm_angle_cmd-angle0)/k;
            %             else
            %                 temp = (arm_angle_cmd-angle180)/k;
            %             end
            % model easily: equation of one unknow, one order
            %             angle0 = 187; % decimalism
            %             angle180 = -3;
            %             k = (angle180-angle0)/(180-0);
            %             temp = (arm_angle_cmd-angle0)/k;
            %             if temp<0
            %                 temp = temp + 180;
            %             elseif temp>180
            %                 temp = temp - 180;
            %             end
            
            obj.arm_angle_cmd = temp;
            cmd = [obj.CMD_MOTOR_HEADER obj.hand_angle_cmd obj.arm_angle_cmd];
            obj.send_list.add(cmd);
        end
        
        function setHandArm_AngleAsync(obj,hand_angle_cmd,arm_angle_cmd)
            CASPR_log.Assert(arm_angle_cmd<= obj.MAX_ARM_ANGLE && arm_angle_cmd>=obj.MIN_ARM_ANGLE,'The argument is out of range!');
            CASPR_log.Assert(hand_angle_cmd<= obj.MAX_HAND_ANGLE && hand_angle_cmd>=obj.MIN_HAND_ANGLE,'The argument is out of range');
            obj.hand_angle_cmd = hand_angle_cmd;
            obj.arm_angle_cmd = arm_angle_cmd;
            cmd = [obj.CMD_MOTOR_HEADER obj.hand_angle_cmd obj.arm_angle_cmd];
            obj.send_list.add(cmd);
        end
        
        function us_distance = getUltraSonic(obj)
            us_distance = obj.us_distance;
        end
        
        
        %         % The default value is 1 second. The minimum value is 0.01 second.
        %         TIMERPERIOD = 0.05;%s
        function timer_callback(obj)
            %             if(obj.us_enable)
            %                 if(obj.us_timer == 0)
            %                     % upload the us cmd
            %                     obj.send_list.add(double(obj.CMD_DISTANCE_READ));
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
                obj.sendCMDAsync(obj.send_list.remove());
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
        %                 obj.sendCMDAsync(obj.send_list.remove());
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
            %            stopasync(obj.comPort);
            %            pause(1);
            fclose(obj.comPort);
            delete(obj.comPort);
            clear obj.comPort;
        end
        
        function sendReadCMDAsync(obj)
            obj.send_list.add(obj.CMD_DISTANCE_READ);
        end
        
        function sendCMDAsync(obj, cmd)
            if(strcmpi(obj.comPort.TransferStatus,'read') || strcmpi(obj.comPort.TransferStatus,'idle'))
                cmd_intermedia = uint8(cmd);
                fwrite(obj.comPort,cmd_intermedia,'async');
            end
        end
        
        function [arm_angle_cmd] = armAngleConvert(obj, arm_angle_needed)
            CASPR_log.Assert(arm_angle_needed<= obj.MAX_ARM_ANGLE && arm_angle_needed>=obj.MIN_ARM_ANGLE,'The argument is out of range!');
            
%             if(arm_angle_needed<5)
%                 arm_angle_needed = arm_angle_needed + 180;
%             end
            if(arm_angle_needed>106)
                arm_angle_needed = arm_angle_needed - 180;
            end
            
            arm_angle_cmd = interp1(obj.LOOKUP_ARM_ANGLE(:,2),obj.LOOKUP_ARM_ANGLE(:,1),arm_angle_needed);
            
            % interpolation testing
            %             figure;
            %             x = (1:180)';
            %             x(find(x>=95)) = x(find(x>=95)) - 180;
            %             y = interp1(LOOKUP_ARM_ANGLE(:,2),LOOKUP_ARM_ANGLE(:,1),x,'PCHIP');
            %             plot(LOOKUP_ARM_ANGLE(:,2),LOOKUP_ARM_ANGLE(:,1),'o', x, y, 'r');
        end
        
        function [distance] = distanceConvert(obj, intensity)
            distance = zeros(size(intensity));
            distance(1) = interp1(obj.LOOKUP_DISTANCE_SENSOR1(:,1),obj.LOOKUP_DISTANCE_SENSOR1(:,2),intensity(1));
            distance(2) = interp1(obj.LOOKUP_DISTANCE_SENSOR2(:,1),obj.LOOKUP_DISTANCE_SENSOR2(:,2),intensity(2));
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

