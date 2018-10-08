classdef RobotiqGripper < handle
    properties (Access = public)
        current_servo_angle = 90
        current_gripper_state = 1
        
        strComPort
        comPort
        send_list
    end
    
    properties (Access = public, Constant = true)
        BAUDRATE       = 38400;
        SERIAL_TIMEOUT = 2;%s Waiting time to complete a read or write operation
        Gripper_activateRequest = 1;
        Gripper_activate = 2;
        Gripper_CLOSE = 3;
        Gripper_OPEN = 4;

        max_servo_angle  = 180;
        min_servo_angle  = 0;
        ini_servo_angle  = 90;

    end
    
    methods (Access = public)
        function robotiqgripper = RobotiqGripper(strComPort)
            robotiqgripper.strComPort = strComPort;
            robotiqgripper.current_servo_angle  = robotiqgripper.ini_servo_angle;
        end
        
        function initialize(obj)
            obj.comPort = serial(obj.strComPort);
            obj.comPort.BaudRate = obj.BAUDRATE;
            obj.comPort.timeout = obj.SERIAL_TIMEOUT;
            obj.comPort.ReadAsyncMode = 'continuous';
            obj.openserial();

            cmd = sprintf('(%d,%d)',obj.current_servo_angle,obj.Gripper_activateRequest);
            fwrite(obj.comPort,cmd);
            pause(1);
            cmd = sprintf('(%d,%d)',obj.current_servo_angle,obj.Gripper_activate);
            fwrite(obj.comPort,cmd);
        end


        function disconnect(obj)
            obj.closeserial();
        end
        
        function setGripper(obj,gripperState)
            %             CASPR_log.Assert(hand_angle_cmd<= obj.MAX_HAND_ANGLE && hand_angle_cmd>=obj.MIN_HAND_ANGLE,'The argument is out of range!');
            obj.current_gripper_state = gripperState;
            cmd = sprintf('(%d,%d)',obj.current_servo_angle,obj.current_gripper_state);
            fwrite(obj.comPort,cmd);
        end
        
        function setServo(obj,servoAngle)
            obj.current_servo_angle = servoAngle;
            cmd = sprintf('(%d,%d)',obj.current_servo_angle,obj.current_gripper_state);
            fwrite(obj.comPort,cmd);
        end
      end
    
    methods (Access = private)
        function openserial(obj)
            fopen(obj.comPort);
        end
        
        function closeserial(obj)
            fclose(obj.comPort);
            delete(obj.comPort);
            clear obj.comPort;
        end
    end
end


