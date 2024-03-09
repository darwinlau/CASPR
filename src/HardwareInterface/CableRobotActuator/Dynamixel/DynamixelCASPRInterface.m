% All length variables in this class use metric unit.
classdef DynamixelCASPRInterface < CableActuatorInterfaceBase
    properties (Access = private)
        numMotor            % Number of actuator
        comPort             % Serial COM port connected to USB2Dynamixel
        numPort             % The number of COM port
        
        
        lib_name
        
        
        % including every set of the spool and the dynamixel holders
        accessories
        
        cableLengths_initial       % size: numMotor X 1
        cableLengths_full          % size: numMotor X 1
%         dynamixel_position_initial % size: numMotor X 1
        
        dynamixel_direction_factor_position
        dynamixel_direction_factor_current
        dynamixel_direction_reversed
        
        % Protocol version
        PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel
        BAUDRATE                    = 115200
        
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        MAX_ID                      = 252;          % Maximum ID value
    end
    
    properties (Access = public)
        port_num            % The generated ID for each COM Port
        % DXL_ID is a column vector including the IDs of all motors
        DXL_ID;
        % The number of the motors found
        DXL_NUM;
        
        % The default KpD, KpI and KpP
        KpD;% KPD = KPD(TBL) / 16
        KpI;% KPI = KPI(TBL) / 65536
        KpP;%   = KPP(TBL) / 128
        dynamixel_position_initial % TEMPORARILY in public
        loosenFactor
        ActuatorParas
        gripper
    end
    
    methods (Access = public)
        % e.g. comPort = 'COM3'
        % cableLengths_full: SIZE: numMotor x 1
        function interface = DynamixelCASPRInterface(comPort, actuatorType, numMotor, dynamixel_direction_reversed, gripperOn)
            interface@CableActuatorInterfaceBase();
            if(~iscellstr(comPort))
                CASPR_Log.Error('The argument ''comport'' should be a cell array of character arrays!');
            end
                switch actuatorType
                    case DynamixelType.XH430_W210
                        interface.ActuatorParas = XH430_W210;
                    case DynamixelType.XM540_W150
                        interface.ActuatorParas = XM540_W150;
                    case DynamixelType.PRO_M54_60_S250
                        interface.ActuatorParas = PRO_M54_60_S250;
                    case DynamixelType.PRO_H54P_200_S500
                        interface.ActuatorParas = PRO_H54P_200_S500;
                    case DynamixelType.RH_P12_RN
                        interface.ActuatorParas = RH_P12_RN;
                    otherwise
                        CASPR_Log.Error('ActuatorType has not been implemented');
                end
            interface.numPort = length(comPort);
            interface.DXL_ID = cell(interface.numPort,1);
            interface.DXL_NUM = zeros(interface.numPort,1);
            interface.comPort = comPort;
            interface.numMotor = numMotor;
            interface.dynamixel_direction_reversed = dynamixel_direction_reversed;
            if(nargin >=3 && exist('gripperOn','var'))
                interface.gripper = gripperOn;
            else
                interface.gripper = 0;
            end
            if(dynamixel_direction_reversed)
                interface.dynamixel_direction_factor_position = -1;
                interface.dynamixel_direction_factor_current  = 1;
            else
                interface.dynamixel_direction_factor_position = 1;
                interface.dynamixel_direction_factor_current  =-1;
            end
            if actuatorType == DynamixelType.XH430_W210
                for i = 1:numMotor
                    accessories_temp(i) = SmallSpoolSpecifications;
                end
            elseif actuatorType == DynamixelType.XM540_W150 || actuatorType == DynamixelType.PRO_M54_60_S250
                for i = 1:numMotor
                    accessories_temp(i) = MegaSpoolSpecifications;
                end
            elseif actuatorType == DynamixelType.PRO_H54P_200_S500
                for i = 1:numMotor
                    accessories_temp(i) = meter_20_Spool;
                end
            else
                disp('Please specify spool type in DynamixelCASPRInterface')
                % TODO put the spool type into xml for spool types
            end
            interface.accessories = accessories_temp;
            interface.cableLengths_full = accessories_temp(1).cableLengths_full;
            interface.initialise;
        end
    end
    
    %     methods (Static)
    %         function singleObj = getInstance
    %             persistent localObj
    %             if isempty(localObj) || ~isvalid(localObj)
    %                 localObj = PoCaBotCASPRInterface;
    %             end
    %             singleObj = localObj;
    %         end
    %     end
    
    methods
        % comPort = 'COM3'
       
    end
    
    methods
         function initialise(obj)
            % Find and Load Libraries
            if strcmp(computer, 'PCWIN')
                libname = 'dxl_x86_c';
            elseif strcmp(computer, 'PCWIN64')
                libname = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
                libname = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
                libname = 'libdxl_x64_c';
            end
            obj.lib_name = libname;
            
            % NOTE: If any error occurs here, please first check whether
            % the dll need to be re-compiled. If needed, change directory to
            % ./c/build/, choose the approriate folder according to your
            % operating system. And re-compile these source codes with
            % relevant files.
            if ~libisloaded(obj.lib_name)
                loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
            end
            
            % Initialize PortHandler Structs
            obj.port_num = [];
            for i = 1:obj.numPort                
                obj.port_num(i) = portHandler(char(obj.comPort(i)));
            end
            
            % Initialize PacketHandler Structs
            packetHandler();
        end
        
        function open(obj)
            % Initialize the serialpush port communication between Dynamixel and MATLAB
            % The input value is the COMPORT should be changed as per requirement
            % We ensure that the Dynamixel is also communicatiing with MATLAB at this
            % time. A predefined code on the arduino acknowledges this.
            if ~libisloaded(obj.lib_name)
                loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
            end
            
            for i = 1:obj.numPort
                if (openPort(obj.port_num(i)))
                    CASPR_log.Debug('Succeeded to open the port!\n');
                else
                    unloadlibrary(obj.lib_name);
                    CASPR_log.Error('Failed to open the port!\n');
                end
            end
            
            % Set port baudrate
            for i = 1:obj.numPort
                if (setBaudRate(obj.port_num(i), obj.BAUDRATE))
                    CASPR_log.Debug('Succeeded to change the baudrate!\n');
                else
                    unloadlibrary(obj.lib_name);
                    CASPR_log.Error('Failed to change the baudrate!\n');
                end
            end
        end
        
        function close(obj)
            % Close port
            for i = 1:obj.numPort
                closePort(obj.port_num(i));
            end
            
            % Unload Library
            unloadlibrary(obj.lib_name);
        end
        
        % Initialise the ID of Dynamixels, before this procedure, we
        % need to check the ID with R+ manager 2.0 first to avoid IDs
        % repeat.
        % Sends ping cmd to the devices and waits for the response
        % CAUTION: WHEN THE ID IS ZERO, IT CANNOT BE DETECTED BY broadcastPING!
        % THIS IS VERIFIED BY TESTING.
        function [success] =detectDevice(obj)
            success = 0;
            % Try to broadcast ping the Dynamixels
            for i = 1:obj.numPort-obj.gripper
                broadcastPing(obj.port_num(i), obj.PROTOCOL_VERSION);
                for id = 0 : obj.MAX_ID
                    if getBroadcastPingResult(obj.port_num(i), obj.PROTOCOL_VERSION, id)
                        obj.DXL_NUM(i) = obj.DXL_NUM(i) + 1;
                        obj.DXL_ID(i) = {[cell2mat(obj.DXL_ID(i)); id]};
                    end
                end
            end
%             obj.DXL_NUM = [4;4;2];
%             obj.DXL_ID{1} = [1,6,7,8];
%             obj.DXL_ID{2} = [2,3,4,5];
            if(obj.numMotor ~= sum(obj.DXL_NUM))
%                 obj.close();
%                 CASPR_log.INFO('The number of the motors detected does not match the number you specified');
            disp('The number of the motors detected does not match the number you specified');
            end
            
            if(sum(obj.DXL_NUM) > 0)
                success = 1;
            end
        end
        
        % Method to send the vector of cable lengths (l_cmd) to hardware
        %
        % Argument l_cmd is a column vector with size (numMotor x 1)
        %
        % Make sure that the dynamixels have been set to the extended
        % position mode or Current-based Position Control Mode
        % before invoking this function.
        function lengthCommandSend(obj, l_cmd)
            if(length(l_cmd) ~= obj.numMotor)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument l0 and try again');
            end
            [dynamixel_position_cmd] = obj.len2anglecmd(l_cmd);
            obj.sync_write(obj.ActuatorParas.ADDR_GOAL_POSITION, obj.ActuatorParas.LEN_GOAL_POSITION, round(dynamixel_position_cmd));
        end
        
        % Method to send the initial cable length information to the
        % hardware interface and hardware (if required)
        %
        % For application of dynamixels, this funciton is used to specify
        % the initial length from the motor model outlet point to the
        % correspondnig attachment point on the end effector.
        % The size of l0 here should be numMotor X 1. The elements in l0 is
        % the absolute length, so they should be positive.
        function lengthInitialSend(obj, l0)
            if(length(l0) ~= obj.numMotor)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument l0 and try again');
            end
            obj.cableLengths_initial = l0;
            
            deltaLengths_init = obj.cableLengths_full - obj.cableLengths_initial;
            for i = 1:obj.numMotor
                obj.accessories(i).setInitState(deltaLengths_init(i));
            end
        end
        
        % Method to send the initial motor position information to the
        % hardware interface and hardware (as appropriate)
        %
        % For application of dynamixels, this funciton is used to specify
        % the initial  motor position which is the real position where
        % end effector is in its initial position and the cable is under no
        % tension.
        % The size of p0 here should be numMotor X 1. The elements in p0 is
        % the absolute position.
        function motorPosInitialSend(obj,p0)
            obj.dynamixel_position_initial = p0;
        end
        
        function motorPositionCommandSend(obj, p_cmd)
            obj.sync_write(obj.ActuatorParas.ADDR_GOAL_POSITION, obj.ActuatorParas.LEN_GOAL_POSITION, p_cmd);
        end
        
        function [position] = motorPositionFeedbackRead(obj)
            [~, position] = obj.sync_read(obj.ActuatorParas.ADDR_PRESENT_POSITION, obj.ActuatorParas.LEN_PRESENT_POSITION);
        end
        
        % Method to read the cable lengths from the hardware (if available)
        function [length] = relativelengthFeedbackRead(obj, initial)
            [bState, ret] = obj.sync_read(obj.ActuatorParas.ADDR_PRESENT_POSITION, obj.ActuatorParas.LEN_PRESENT_POSITION);
            if(bState)
                deltaAngle = (ret - initial)/obj.ActuatorParas.ENCODER_COUNT_PER_TURN*2*pi;
                deltalength = zeros(size(deltaAngle));
                for i = 1:obj.numMotor
%                     deltalength(i) = obj.dynamixel_direction_factor_position * obj.accessories(i).getDeltaLength(deltaAngle(i));
                
                end
                    length = deltalength();
            else
                length = ones(size(ret))*(-1);
            end            
        end
        function [length] = lengthFeedbackRead(obj)
            [bState, ret] = obj.sync_read(obj.ActuatorParas.ADDR_PRESENT_POSITION, obj.ActuatorParas.LEN_PRESENT_POSITION);
            if(bState)
                deltaAngle = (ret - obj.dynamixel_position_initial)/obj.ActuatorParas.ENCODER_COUNT_PER_TURN*2*pi;
                deltalength = zeros(size(deltaAngle));
                for i = 1:obj.numMotor
                    deltalength(i) = obj.dynamixel_direction_factor_position * obj.accessories(i).getDeltaLength(deltaAngle(i));
                end
                length = obj.cableLengths_initial + deltalength;
            else
                length = ones(size(ret))*(-1);
            end            
        end
        % Method to send some "on" state to the hardware (optional)
        % Here used to enable the torque of the dynamixel to make it
        % controllable.
        function systemOnSend(obj)
            obj.toggleEnableAllDynamixel(obj.TORQUE_ENABLE);
        end
        
        % Method to send some "off" state to the hardware (optional)
        function systemOffSend(obj)
            %[current] = obj.forceFeedbackRead();
%             cur_release = 100;
%             current = ones(obj.numMotor,1)*cur_release;
%             obj.forceCommandSend(current);
%             obj.tightenCablesWithinPositionMode();%if it is not in current based position operating mode, it should be fine. Because in this case, this statement shall not work by any means.
%             cnt = 50;
%             for i = 1:cnt
%                 current(1:2:obj.numMotor) = cur_release*(1-i/cnt);
%                 obj.forceCommandSend(current);
%                 pause(0.1);
%             end
%             
%             for i = 1:cnt
%                 current(2:2:obj.numMotor) = cur_release*(1-i/cnt);
%                 obj.forceCommandSend(current);
%                 pause(0.1);
%             end
%             pause(1);
            obj.toggleEnableAllDynamixel(obj.TORQUE_DISABLE);
        end
        
        function switchOperatingMode(obj,om)
            switch om
                case ActuatorOperatingModeType.CURRENT_MODE
                    obj.switchOperatingMode2CURRENT();
                case ActuatorOperatingModeType.VELOCITY_MODE
                    
                case ActuatorOperatingModeType.POSITION_MODE %Single circle
                case ActuatorOperatingModeType.EXTENDED_POSITION_MODE %Multiple circles
                case ActuatorOperatingModeType.CURRENT_BASED_POSITION_MODE %Multiple circles with constrained current
                    obj.switchOperatingMode2POSITION_LIMITEDCURRENT();
                case ActuatorOperatingModeType.PWM_MODE
                otherwise
            end
        end
%         % Beware of that when trying to modify the operatiing mode, the
%         % motors must be turned off.
%         ADDR_OPERATING_MODE         = 11;
%         LEN_OPERATING_MODE          = 1;
%         
%         OPERATING_MODE_CURRENT   = 0;
%         OPERATING_MODE_CURRENT_BASED_POSITION = 5;
%         Argument operating_mode is OPERATING_MODE_CURRENT or
%         OPERATING_MODE_CURRENT_BASED_POSITION.
        function switchOperatingMode2CURRENT(obj)
            [~, mode] = obj.sync_read(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE);
            if(~all(mode == obj.ActuatorParas.OPERATING_MODE_CURRENT))
                obj.toggleEnableAllDynamixel(obj.TORQUE_DISABLE);
                obj.sync_write(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE, ones(obj.numMotor,1)*obj.ActuatorParas.OPERATING_MODE_CURRENT);
            end
        end
        
        function switchOperatingMode2POSITION_LIMITEDCURRENT(obj)
            [~, mode] = obj.sync_read(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE);
            if(~all(mode == obj.ActuatorParas.OPERATING_MODE_CURRENT_BASED_POSITION))
                obj.toggleEnableAllDynamixel(obj.TORQUE_DISABLE);
                obj.sync_write(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE, ones(obj.numMotor,1)*obj.ActuatorParas.OPERATING_MODE_CURRENT_BASED_POSITION);
            end
        end
        
%         function switchOperatingMode2POSITION_LIMITEDCURRENT_ByMotor(obj,motor)
%             [~, mode] = obj.sync_read(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE);
%             if(mode(motor) ~= obj.ActuatorParas.OPERATING_MODE_EXTENDED_POSITION) || range(mode)==0 % check for condition that should change the op mode of motor
%                 obj.toggleEnableAllDynamixel(obj.TORQUE_DISABLE);
%                 operatingModeVector = ones(obj.numMotor,1)*obj.ActuatorParas.OPERATING_MODE_CURRENT;
% %                 operatingModeVector(motor) = obj.ActuatorParas.OPERATING_MODE_CURRENT_BASED_POSITION
%                 operatingModeVector(motor) = obj.ActuatorParas.OPERATING_MODE_EXTENDED_POSITION; % Using extended position mode for better stiffness
%                 obj.sync_write(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE, operatingModeVector);
%             end
%         end
        
        % This function assign motors with different mode running at the
        % same time. Input motorMode should be a numMotor x1 vector,
        % indicating the targeting operating mode, and current is ranging
        % from 0 to 2047, normally we have 67 as default current
        function hybridOperatingMode(obj,motorMode,current)
            if nargin < 3
                current = ones(obj.numMotor,1)*obj.ActuatorParas.MAX_WORK_CURRENT/15;
            else
                current = ones(obj.numMotor,1)*current;
            end
            currentOperatingMode = obj.getOperatingMode;
            obj.toggleEnableSomeDynamixel(obj.TORQUE_DISABLE, currentOperatingMode(1:obj.numMotor) ~= motorMode);
            obj.sync_write(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE, motorMode);
            obj.systemOnSend();
            obj.forceCommandSend(current);
        end
        
        function [mode] = getOperatingMode(obj)
            [~, mode] = obj.sync_read(obj.ActuatorParas.ADDR_OPERATING_MODE, obj.ActuatorParas.LEN_OPERATING_MODE);
        end
        
        % Method to send the vector of current (f_cmd) to hardware
        %
        % Argument f_cmd is a column vector with size (numMotor x 1)
        function forceCommandSend(obj, f_cmd)
            if(length(f_cmd) ~= obj.numMotor)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument c_cmd and try again');
            end
            obj.sync_write(obj.ActuatorParas.ADDR_GOAL_CURRENT, obj.ActuatorParas.LEN_GOAL_CURRENT, f_cmd*obj.dynamixel_direction_factor_current);
        end
        % Intended for Dynamixel Pro Plus
        function setGoalCurrent(obj, current_cmd)
            if(length(current_cmd) ~= obj.numMotor)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument c_cmd and try again');
            end
            obj.sync_write(obj.ActuatorParas.ADDR_GOAL_CURRENT, obj.ActuatorParas.LEN_GOAL_CURRENT, current_cmd);
        end
        
        % Method to read the current from the hardware (if available)
        function [current] = forceFeedbackRead(obj)
            [~, current] = obj.sync_read(obj.ActuatorParas.ADDR_PRESENT_CURRENT, obj.ActuatorParas.LEN_PRESENT_CURRENT);
            current = current*obj.dynamixel_direction_factor_current;
        end
        
        % Method to read the goal current from the hardware (if available)
        function [current] = goalForceFeedbackRead(obj)
            [~, current] = obj.sync_read(obj.ActuatorParas.ADDR_GOAL_CURRENT, obj.ActuatorParas.LEN_GOAL_CURRENT);
            current = current*obj.dynamixel_direction_factor_current;
        end
        
        function setProfileAcceleration(obj,profile)
            obj.sync_write(obj.ActuatorParas.ADDR_PROFILE_ACCELERATION, obj.ActuatorParas.LEN_PROFILE_ACCELERATION, profile);
        end
        
        function setProfileVelocity(obj,profile)
            obj.sync_write(obj.ActuatorParas.ADDR_PROFILE_VELOCITY, obj.ActuatorParas.LEN_PROFILE_VELOCITY, profile);
        end
        
%         % Tighten the cables without chaning the mode into CURRENT mode.
%         % The precondition is that the motors are working under Extended
%         % Position Mode, no matter with or without current constrained.
        function tightenCablesWithinPositionMode(obj)
            cmd = -1*obj.dynamixel_direction_factor_position*1048574*ones(obj.numMotor,1);
            obj.motorPositionCommandSend(cmd);
        end
        
        % Enable or Disable the motors
        function switchEnable(obj, cmd)
            obj.sync_write(obj.ActuatorParas.ADDR_TORQUE_ENABLE, obj.ActuatorParas.LEN_TORQUE_ENABLE, cmd);
        end
        
        % Set the PID paras of the Position loop
        % The arguments KpP/KpI/KpD are with size numMotor by 1
        function setKpD(obj,KpD)
            obj.KpD = KpD;
            obj.sync_write(obj.ActuatorParas.ADDR_KpD, obj.ActuatorParas.LEN_KpD, KpD);
        end
        function setKpI(obj,KpI)
            obj.KpI = KpI;
            obj.sync_write(obj.ActuatorParas.ADDR_KpI, obj.ActuatorParas.LEN_KpI, KpI);
        end
        function setKpP(obj,KpP)
            obj.KpP = KpP;
            obj.sync_write(obj.ActuatorParas.ADDR_KpP, obj.ActuatorParas.LEN_KpP, KpP);
        end
        % Get the PID parameters of the Position control loop
        function [KpD] = getKpD(obj)
            [~, KpD] = obj.sync_read(obj.ActuatorParas.ADDR_KpD, obj.ActuatorParas.LEN_KpD);
            obj.KpD = KpD;
        end
        function [KpI] = getKpI(obj)
            [~, KpI] = obj.sync_read(obj.ActuatorParas.ADDR_KpI, obj.ActuatorParas.LEN_KpI);
            obj.KpI = KpI;
        end
        function [KpP] = getKpP(obj)
            [~, KpP] = obj.sync_read(obj.ActuatorParas.ADDR_KpP, obj.ActuatorParas.LEN_KpP);
            obj.KpP = KpP;
        end
        
        % This function is used when the motor is working in position mode
        % with the KpI set to zero.
        % offset: the offset length of the cable caused by KpP.
        function [offset] = getCableOffsetByTension(obj, tension)
            % when the supplied voltage is 12V, the stall torque from the
            % manual is 2.5N.m
            % obj.MAX_TORQUE = 1.5    UNIT:N.m; This value of 1.5 N.m was obtained by our experiment.
            maxTension = obj.ActuatorParas.MAX_TORQUE/obj.accessories(1).radius;
            current = tension/maxTension*obj.ActuatorParas.MAX_CURRENT;
            Kp = obj.KpP/obj.ActuatorParas.KpP_SCALE_FACTOR;% KPP = KPP(TBL) / 128
            errAngle = current./Kp;
            offset = -2 * errAngle/obj.ActuatorParas.ENCODER_COUNT_PER_TURN.*[obj.accessories(:).len_per_circle]';
        end
        
        % Below function is written by Ding and designed for AEI CU-Brick
        % model
        function [offset] = IDcableOffset(obj,cableForces, cableLength)
            forceOffsetConstant = 0.001;
            lengthOffset = zeros(length(cableLength),1);
            for i = 1:length(cableLength)
                if cableLength(i) < 0.3 %1.5
                    lengthOffsetConstant = 0.001; %0.003
                    lengthOffset(i) = cableLength(i)*lengthOffsetConstant;
                elseif cableLength(i) > 0.3 %1.5
                    lengthOffsetConstant = 0.003;   %0.005
                    lengthOffset(i) = cableLength(i)*lengthOffsetConstant;
                end
            end

            forceOffset = cableForces.*forceOffsetConstant;
            offset = forceOffset + lengthOffset;
            offset = zeros(size(offset));
            maxOffset = ones(obj.numMotor,1)*0.015;
            minOffset = [0.004;0.004;0.004;0.004;0.004;0.004;0.004;0.004;]*0.5;
%             minOffset = [0.004;0.002;0.004;0.002;0.004;0.002;0.004;0.002;]*0.25;
            for i = 1:length(offset)
                if(offset(i)>maxOffset(i))
%                     fprintf('maxOffset is occured at %d, with size of %3d\n', i, maxOffset(i));
                    offset(i) = maxOffset(i);
                end
                if(offset(i)<minOffset(i))
%                     fprintf('minOffset is occured at %d, with size of %3d\n', i,minOffset(i));
                    offset(i) = minOffset(i);
                end
            end      
        end

        % This function is used to find out which cables in loosed during
        % current mode
        function detectLoosenCables(obj)
            loosenIndex = [];
            obj.loosenFactor = 0.3;
            loosenTension = obj.goalForceFeedbackRead()* obj.loosenFactor;
            cableForceDiff = obj.goalForceFeedbackRead() - obj.forceFeedbackRead();
            currentOM = obj.getOperatingMode;
            if any(abs(cableForceDiff) > loosenTension)
                for i = 1:obj.numMotor
                    if ((abs(cableForceDiff(i))>loosenTension(i)) && currentOM(i)==obj.ActuatorParas.OPERATING_MODE_CURRENT)
                        loosenIndex =[loosenIndex; i];
                    end
                end
                if ~isempty(loosenIndex)
                    if length(loosenIndex) < 2
                        fprintf('Cable %d is loose\n',loosenIndex);
                    else
                        fprintf('Cable')
                        for j = 1:length(loosenIndex)-1
                            fprintf(' %d,', loosenIndex(j));
                        end
                       fprintf('%d are loose',loosenIndex(end))
                    end
                end
            end
        end
        % This function is used when the motor is working in current mode
        % Tension: The approximating force derived from the nominal
        % current.
        function [tension] = getCableTensionByCurrent(obj, current)
            maxTension = obj.ActuatorParas.MAX_TORQUE./[obj.accessories(:).radius]';
            tension = current/obj.ActuatorParas.MAX_CURRENT*maxTension;
        end
        
        % When initialize the system, there is a time when all the cables
        % are tensioned and then we can put the end effector at the
        % specified initial position. This method is to get the motor
        % position offset produced when the cables are tensioned initially.
        % The input argument 'deltaLengh' with size (numMotor x 1) and with 
        % all entries positive is the 
        % cable length offset caused when the cables are tensioned.
        function [motorPosOffset] = getMotorPosInitOffset(obj, deltaLength)
            if(length(deltaLength) ~= obj.numMotor)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument deltaLength and try again');
            end
            motorPosOffset = obj.dynamixel_direction_factor_position*deltaLength./[obj.accessories(:).len_per_circle]'*obj.ActuatorParas.ENCODER_COUNT_PER_TURN;
        end
        
        function [drivemode] = getDriveMode(obj)
            [~, drivemode] = obj.sync_read(obj.ActuatorParas.ADDR_DRIVE_MODE, obj.ActuatorParas.LEN_DRIVE_MODE);
        end
        
        % if hardware_error_status is greater than 0, the dynamixel needs
        % to be rebooted.
        function [hardware_error_status] = getHardwareErrorStatus(obj)
            [~, hardware_error_status] = obj.sync_read(obj.ActuatorParas.ADDR_HARDWARE_ERROR_STATUS, obj.ActuatorParas.LEN_HARDWARE_ERROR_STATUS);
        end
        
        function [actuatorParas] = getActuatorParas(obj)
            actuatorParas = obj.ActuatorParas;
        end
    end
    
    methods (Access = public)
        % This function has built in the math model of the spool and the
        % dynamixel holder. the input argument relLen is a column vector
        % with size numMotor X 1, specifying the absolute length we need
        % to achieve by pulling or loosing the cables.
        function [dynamixel_position_cmd] = len2anglecmd(obj, relLen)
            if(length(relLen) ~= obj.numMotor)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument relLen and try again');
            end
            delta_lengths = relLen - obj.cableLengths_initial;
            for i = 1:obj.numMotor
                angle_delta(i) = obj.accessories(i).getDeltaAngle(delta_lengths(i));
            end
            dynamixel_position_delta = obj.dynamixel_direction_factor_position * angle_delta/2/pi*obj.ActuatorParas.ENCODER_COUNT_PER_TURN;
            dynamixel_position_cmd = obj.dynamixel_position_initial + dynamixel_position_delta';
        end
        
        function [dynamixel_position_cmd] = relLen2anglecmd(obj, delta_length, currentPose, motor)
            angle_delta = deg2rad(delta_length/obj.accessories(1, 1).len_per_circle*360);
            dynamixel_position_delta = obj.dynamixel_direction_factor_position * angle_delta/2/pi*obj.ActuatorParas.ENCODER_COUNT_PER_TURN;
            dynamixel_position_delta_vector = zeros(obj.numMotor,1);
            dynamixel_position_delta_vector(motor) = dynamixel_position_delta;
            dynamixel_position_cmd = currentPose + dynamixel_position_delta_vector;
        end
        
        function [pos] = radian2dynamixelposition(obj, angle)
            if(~isvector(angle))
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument angle_delta and try again');
            end
            pos = angle/2/pi*obj.ActuatorParas.ENCODER_COUNT_PER_TURN;
        end
        
        % cmd could be TORQUE_ENABLE or TORQUE_DISABLE
        function toggleEnableSingleDynamixel(obj, dxl_ID, cmd)
            for i = 1:obj.numPort
                if(any(cell2mat(obj.DXL_ID(i)) == dxl_ID))
                    break;
                end
            end
            write1ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, dxl_ID, obj.ActuatorParas.ADDR_TORQUE_ENABLE, cmd);
            if( getLastTxRxResult(obj.port_num(i), obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num(i), obj.PROTOCOL_VERSION) == 0 )
                CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully controlled \n', dxl_ID));
            end
        end
        
        % cmd could be TORQUE_ENABLE or TORQUE_DISABLE
        function toggleEnableAllDynamixel(obj, cmd)
            for i = 1:obj.numPort-obj.gripper
                currentID_group = cell2mat(obj.DXL_ID(i));
                for dxl = 1:obj.DXL_NUM(i)
                    write1ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, currentID_group(dxl), obj.ActuatorParas.ADDR_TORQUE_ENABLE, cmd);
                    if( getLastTxRxResult(obj.port_num(i), obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num(i), obj.PROTOCOL_VERSION) == 0 )
                        CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully controlled \n', currentID_group(dxl)));
                    end
                end
            end
        end
        % This function is a variation of the All enabling function, which
        % used to disable or enable the torque on selected motors, motor is
        % of size obj.numMotor x 1, as a boolean object that selecting the
        % motors in operation
        function toggleEnableSomeDynamixel(obj, cmd, motor)
            for i = 1:obj.numPort-obj.gripper
                currentID_group = cell2mat(obj.DXL_ID(i));
                for dxl = 1:obj.DXL_NUM(i)
                    if motor(currentID_group(dxl))
                        write1ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, currentID_group(dxl), obj.ActuatorParas.ADDR_TORQUE_ENABLE, cmd);
                    else
                    end
                end
            end
        end
        
        % write into a single motor
        function single_write(obj, dxl_ID, start_address, data_length, data)
            for i = 1:obj.numPort
                if(any(cell2mat(obj.DXL_ID(i)) == dxl_ID))
                    break;
                end
            end
            switch data_length
                case 1
                    write1ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, dxl_ID, start_address, typecast(int8(data), 'uint8'));
                    if( getLastTxRxResult(obj.port_num(i), obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num(i), obj.PROTOCOL_VERSION) == 0 )
                        CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully written in \n', dxl_ID));
                    end
                case 2
                    write2ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, dxl_ID, start_address, typecast(int16(data), 'uint16'));
                    if( getLastTxRxResult(obj.port_num(i), obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num(i), obj.PROTOCOL_VERSION) == 0 )
                        CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully written in \n', dxl_ID));
                    end
                case 4
                    write4ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, dxl_ID, start_address, typecast(int32(data), 'uint32'));
                    if( getLastTxRxResult(obj.port_num(i), obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num(i), obj.PROTOCOL_VERSION) == 0 )
                        CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully written in \n', dxl_ID));
                    end
                otherwise
                    obj.close();
                    CASPR_log.Error('Please input applicable data length!');
            end
        end
        
        % read from a single motor
        function [ret] = single_read(obj, dxl_ID, start_address, data_length)
            for i = 1:obj.numPort
                if(any(cell2mat(obj.DXL_ID(i)) == dxl_ID))
                    break;
                end
            end
            switch data_length
                case 1
                    rep = read1ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, dxl_ID, start_address);
                    ret = typecast(uint8(rep), 'int8');
                case 2
                    rep = read2ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, dxl_ID, start_address);
                    ret = typecast(uint16(rep), 'int16');
                case 4
                    rep = read4ByteTxRx(obj.port_num(i), obj.PROTOCOL_VERSION, dxl_ID, start_address);
                    ret = typecast(uint32(rep), 'int32');
                otherwise
                    obj.close();
                    CASPR_log.Error('Please input applicable data length!');
            end
        end
        
        % argument data_input is a column vector with size (numMotor x 1) and the size of each element
        % of the data cannot exceed 4 bytes.
        function sync_write(obj, start_address, data_length, data_input)
            % Prepare the parameters
            switch data_length
                case 1
                    data = typecast(int8(data_input), 'uint8');
                case 2
                    data = typecast(int16(data_input), 'uint16');
                case 4
                    data = typecast(int32(data_input), 'uint32');
                otherwise
                    obj.close();
                    CASPR_log.Error('Please input applicable data length!');
            end
            
            if(size(data_input) ~= size(data))
                obj.close();
                CASPR_log.Error('Please input matched data length and data!');
            end
            for i = 1:obj.numPort-obj.gripper
                currentID_group = cell2mat(obj.DXL_ID(i));
                % Initialize Groupsyncwrite Structs
                groupwrite_num = groupSyncWrite(obj.port_num(i), obj.PROTOCOL_VERSION, start_address, data_length);
                for dxl = 1:obj.DXL_NUM(i)
                    % Add Dynamixel goal current value to the Syncwrite storage
                    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, currentID_group(dxl), data(currentID_group(dxl)), data_length);
                    if dxl_addparam_result ~= true
                        obj.close();
                        CASPR_log.Error(sprintf('[ID:%03d] groupSyncWrite addparam failed', currentID_group(dxl)));
                    end
                end
                
                
                groupSyncWriteTxPacket(groupwrite_num);
                
                % Clear syncwrite parameter storage
                groupSyncWriteClearParam(groupwrite_num);
            end
        end
        
        % bState = true means successful
        function [bState, ret] = sync_read(obj, start_address, data_length)
            ret = zeros(obj.numMotor,1);
            for i = 1:obj.numPort-obj.gripper
                currentID_group = cell2mat(obj.DXL_ID(i));
                % Prepare the parameters
                groupread_num = groupSyncRead(obj.port_num(i), obj.PROTOCOL_VERSION, start_address, data_length);
                for dxl = 1:obj.DXL_NUM(i)
                    dxl_addparam_result = groupSyncReadAddParam(groupread_num, currentID_group(dxl));
                    if dxl_addparam_result ~= true
                        obj.close();
                        CASPR_log.Error(sprintf('[ID:%03d] groupSyncRead addparam failed', currentID_group(dxl)));
                    end
                end
                groupSyncReadTxRxPacket(groupread_num);
                
                bState = true;
                for dxl = 1:obj.DXL_NUM(i)
                    % Check if groupsyncread data of Dynamixels is available
                    dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, currentID_group(dxl), start_address, data_length);
                    if dxl_getdata_result ~= true
                        CASPR_log.Warn(sprintf('[ID:%03d] groupSyncRead getdata failed', currentID_group(dxl)));
                        bState = false;
                        break;
                    else
                        rep = groupSyncReadGetData(groupread_num, currentID_group(dxl), start_address, data_length);
                        switch data_length
                            case 1
                                ret(currentID_group(dxl)) = typecast(uint8(rep), 'int8');
                            case 2
                                ret(currentID_group(dxl)) = typecast(uint16(rep), 'int16');
                            case 4
                                ret(currentID_group(dxl)) = typecast(uint32(rep), 'int32');
                            otherwise
                                obj.close();
                                CASPR_log.Error('Please input applicable data length!');
                        end
                    end
                end
                groupSyncReadClearParam(groupread_num);
            end
        end

        function CheckCablesInTension(obj, motorInPosMode)
            loosenIndex = [];
            obj.loosenFactor = 0.3;
            loosenTension = obj.goalForceFeedbackRead()* obj.loosenFactor;
            cableForceDiff = obj.goalForceFeedbackRead() - obj.forceFeedbackRead();
            if any(abs(cableForceDiff) > loosenTension)
                for i = 1:obj.model.numCables
                    if ((abs(cableForceDiff(i))>loosenTension(i)) && i~=motorInPosMode)
                        loosenIndex =[loosenIndex; i];
                    end
                end
                if ~isempty(loosenIndex)
                    if length(loosenIndex) < 2
                        fprintf('Cable %d is loose\n',loosenIndex);
                    else
                        fprintf('Cable')
                        for j = 1:length(loosenIndex)-1
                            fprintf(' %d,', loosenIndex(j));
                        end
                       fprintf('%d are loose',loosenIndex(end))
                    end
                end
            end
        end
    end
end