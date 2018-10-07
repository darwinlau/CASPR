classdef RobotiqGripper < handle
    properties (Access = public)
        hand_angle_cmd = Gripper.LOOSE_HAND_ANGLE;
        arm_angle_cmd = 90;
        current_servo_angle
        current_gripper_state
        
        strComPort
        comPort
        send_list
    end
    
    properties (Access = public, Constant = true)
        BAUDRATE       = 115200;
        SERIAL_TIMEOUT = 2;%s Waiting time to complete a read or write operation
        Gripper_activateRequest = 1;
        Gripper_activate = 2;
        Gripper_CLOSE = 3;
        Gripper_OPEN = 4;

        MAX_ARM_ANGLE  = 180;
        MIN_ARM_ANGLE  = 0;
        INI_ARM_ANGLE  = 90;

    end
    
    methods (Access = public)
        function gripper = Gripper(strComPort)
            gripper.strComPort = strComPort;
            gripper.arm_angle_cmd  = gripper.INI_ARM_ANGLE;
        end
        
        function initialize(obj)
            obj.comPort = serial(obj.strComPort);
            obj.comPort.BaudRate = obj.BAUDRATE;
            obj.comPort.timeout = obj.SERIAL_TIMEOUT;
            obj.comPort.ReadAsyncMode = 'continuous';
            obj.openserial();
        end


        function disconnect(obj)
            obj.closeserial();
        end
        
        function setGripper(obj,hand_angle_cmd)
            %             CASPR_log.Assert(hand_angle_cmd<= obj.MAX_HAND_ANGLE && hand_angle_cmd>=obj.MIN_HAND_ANGLE,'The argument is out of range!');
            obj.hand_angle_cmd = hand_angle_cmd;
            cmd = [obj.CMD_MOTOR_HEADER obj.hand_angle_cmd obj.arm_angle_cmd];
            fwrite(obj.comPort,cmd);
        end
        
        function setServo(obj,arm_angle_needed)
            obj.arm_angle_cmd = obj.armAngleConvert(arm_angle_needed);
            cmd = [obj.CMD_MOTOR_HEADER obj.hand_angle_cmd obj.arm_angle_cmd];
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


