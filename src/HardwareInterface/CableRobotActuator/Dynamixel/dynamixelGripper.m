classdef dynamixelGripper < handle
    properties (SetAccess = public)
        % These properties comply with Dynamixel Protocol 2.0
        % Dynamixel Gripper RH-P12-RN
        % Control table address
        ADDR_TORQUE_ENABLE_GRIPPER          = 562;         
        ADDR_GOAL_POSITION_GRIPPER          = 596;        
        ADDR_PRESENT_POSITION_GRIPPER       = 611;
        ADDR_OPERATING_MODE_GRIPPER         = 11;
        ADDR_GOAL_CURRENT_GRIPPER           = 604;
        ADDR_PRESENT_CURRENT_GRIPPER        = 621;    
        
        % Dynamixel Gripper XH430-W210
        % Control table address       
        ADDR_TORQUE_ENABLE          = 64;         % Control table address is different in Dynamixel model
        ADDR_GOAL_POSITION          = 116;
        ADDR_PRESENT_POSITION       = 132;
        ADDR_OPERATING_MODE         = 11;
        ADDR_GOAL_CURRENT           = 102; 
        ADDR_PRESENT_CURRENT        = 126;
        
        ADDR_KPGAIN                 = 84;
      

        % Parameters in common           
        LEN_TORQUE_ENABLE          = 1;         % Control table address is different in Dynamixel model
        LEN_GOAL_POSITION          = 4;
        LEN_PRESENT_POSITION       = 4;
        LEN_OPERATING_MODE         = 1;
        LEN_GOAL_CURRENT           = 2; 
        LEN_PRESENT_CURRENT        = 2;        
 
        OPERATING_MODE_CURRENT = 0;
        OPERATING_MODE_VELOCITY = 1;
        OPERATING_MODE_POSITION = 3; %4096
        OPERATING_MODE_EXTENDED_POSITION = 4;
        OPERATING_MODE_CURRENT_BASED_POSITION = 5;
        OPERATING_MODE_PWM = 16;

        % Protocol version
        PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

        % Default setting
        NUM_MOTOR                   = 2;
        DXL_ID                      = [9;10];
        BAUDRATE                    = 115200;
        DEVICENAME;                                 % Check which port is being used on your controller
                                                    % ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = 1024;      % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 3072;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        GRIPPER_OPEN                = 0;
        GRIPPER_CLOSE               = 300;
        GRIPPER_FULLCLOSE           = 840;
        GRIPPER_AVAILABLE_ANGLE     = 180;
        SERVO_ANGLE_MAX             = 180;
        SERVO_ANGLE_MIN             = 0;
        
        ESC_CHARACTER               = 'e';          % Key for escaping loop

        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        lib_name                    = '';
        index = 1;
        dxl_goal_position           = 0;
        dxl_comm_result;

        dxl_error = 0;                              % Dynamixel error
        dxl_present_position = 0;                   % Present position
        port_num;
    end
    
    methods
        function interface = dynamixelGripper(port_num)
            interface.dxl_comm_result = interface.COMM_TX_FAIL;           % Communication result
            interface.dxl_goal_position = [interface.DXL_MINIMUM_POSITION_VALUE interface.DXL_MAXIMUM_POSITION_VALUE];         % Goal position
            interface.port_num = port_num;
        end
        
        function initialiseGripper(obj)
%             obj.loadLibrary;
%             obj.port_num = portHandler(obj.DEVICENAME);
%             obj.port_num
%             obj.port_num = 2;
%             packetHandler();
            % Open port
%             obj.openComport();
            % Set port baudrate
%             obj.setBaud();
            % Set Operating mode to Extended position
            obj.setOperationMode(ones(obj.NUM_MOTOR,1)*obj.OPERATING_MODE_CURRENT_BASED_POSITION);
            % Enable Dynamixel Torque
            obj.enableTorque();
        end
        function terminateGripper(obj)
            obj.disableTorque();
            % Close port
            closePort(obj.port_num);
            % Unload Library
%             unloadlibrary(obj.lib_name);
        end
        
        function loadLibrary(obj)
            if strcmp(computer, 'PCWIN')
              obj.lib_name = 'dxl_x86_c';
            elseif strcmp(computer, 'PCWIN64')
              obj.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
              obj.lib_name = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
              obj.lib_name = 'libdxl_x64_c';
            end
            

            % Load Libraries
            if ~libisloaded(obj.lib_name)
                [~, ~] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end

        end
        function openComport(obj)
            if (openPort(obj.port_num))
                fprintf('Succeeded to open the port!\n');
            else
                unloadlibrary(obj.lib_name);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end
        end
        function setBaud(obj)
            if (setBaudRate(obj.port_num, obj.BAUDRATE))
                fprintf('Succeeded to change the baudrate!\n');
            else
                unloadlibrary(obj.lib_name);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end
        end
        function enableTorque(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_TORQUE_ENABLE_GRIPPER, obj.TORQUE_ENABLE);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            else
%                     fprintf('Dynamixel has been successfully connected \n');
            end
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_TORQUE_ENABLE, obj.TORQUE_ENABLE);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            else
%                     fprintf('Dynamixel has been successfully connected \n');
            end
        end
        function setOperationMode(obj,operatingMode)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_OPERATING_MODE_GRIPPER, operatingMode(1));
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            else
%                     fprintf('Dynamixel has been changed mode to extPos \n');
            end
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_OPERATING_MODE, operatingMode(2));
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            else
%                     fprintf('Dynamixel has been changed mode to extPos \n');
            end
        end
        function disableTorque(obj)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_TORQUE_ENABLE_GRIPPER, obj.TORQUE_DISABLE);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
           write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
        end
        function setGoalPose(obj, dxl_goal_position)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_GOAL_POSITION_GRIPPER, typecast(int32(dxl_goal_position(1)), 'uint32'));
            obj.dxl_goal_position(1) = dxl_goal_position(1);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_GOAL_POSITION, typecast(int32(dxl_goal_position(2)), 'uint32'));
            obj.dxl_goal_position(2) = dxl_goal_position(2);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
        end
        function setCurrentGoal(obj, goal_current)
            write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_GOAL_CURRENT_GRIPPER, typecast(int32(goal_current(1)), 'uint32'));
%                 obj.dxl_goal_current(i) = goal_current(i);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
            write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_GOAL_CURRENT, typecast(int32(goal_current(2)), 'uint32'));
%                 obj.dxl_goal_current(i) = goal_current(i);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
        end
        function readCurrentPose(obj,showFb)
            obj.dxl_present_position(1) = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_PRESENT_POSITION_GRIPPER);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
            obj.dxl_present_position(1) = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_PRESENT_POSITION);
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
            if showFb
                disp(obj.dxl_present_position);
            end
        end
        function setGripperOpen(obj)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_GOAL_POSITION_GRIPPER, obj.GRIPPER_OPEN);
            obj.dxl_goal_position(1) = obj.GRIPPER_OPEN;
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
        end
        
        function setGripperClose(obj)            
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_GOAL_POSITION_GRIPPER, obj.GRIPPER_CLOSE);
            obj.dxl_goal_position(1) = obj.GRIPPER_CLOSE;
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
        end
        function setGripperFullClose(obj)            
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), obj.ADDR_GOAL_POSITION_GRIPPER, obj.GRIPPER_FULLCLOSE);
            obj.dxl_goal_position(1) = obj.GRIPPER_CLOSE;
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
        end
        function setServoAngle(obj,angle)
            if~(angle<obj.SERVO_ANGLE_MIN || angle > obj.SERVO_ANGLE_MAX) 
            goalPosition = angle * ((obj.DXL_MAXIMUM_POSITION_VALUE - obj.DXL_MINIMUM_POSITION_VALUE)/180) + obj.DXL_MINIMUM_POSITION_VALUE;
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_GOAL_POSITION, typecast(int32(goalPosition), 'uint32'));
            obj.dxl_goal_position(2) = goalPosition;
            if getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION));
            end
            else
                disp('Angle out of range!')
            end
        end
        function setKpgain(obj,gain)
            write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(2), obj.ADDR_KPGAIN, typecast(int32(gain), 'uint32'));
            disp('Kp changed!')
        end
        
        function initialTest(obj)
            obj.setServoAngle(0);
            pause(0.5);
            obj.setServoAngle(180);
            pause(0.5);
            obj.setGripperClose();
            pause(0.5);
            obj.setGripperOpen();
            pause(0.5);
        end
    end
end

