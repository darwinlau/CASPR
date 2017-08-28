% All length variables in this class use metric unit.
classdef PoCaBotCASPRInterface < CableActuatorInterfaceBase
    properties (Access = private)
        numCmd              % Number of actuator command values to send and feedback
        comPort             % Serial COM port connected to USB2Dynamixel
        
        
        lib_name
        port_num
        
        % including every set of the spool and the dynamixel holders
        accessories
        
        cableLengths_initial       % size: DXL_NUM X 1
        cableLengths_full          % size: DXL_NUM X 1
        dynamixel_position_initial % size: DXL_NUM X 1
        
        dynamixel_direction_factor_position
        dynamixel_direction_factor_current
        dynamixel_direction_reversed
    end
    
    properties (Access = public)
        % DXL_ID is a column vector including the IDs of all motors
        DXL_ID;
        % The number of the motors found
        DXL_NUM;
        
        % The default KpD, KpI and KpP
        KpD = 0;% KPD = KPD(TBL) / 16
        KpI = 0;% KPI = KPI(TBL) / 65536
        KpP = 900;% KPP = KPP(TBL) / 128
    end
    
    properties (Access = private, Constant = true)
        
        % These properties comply with Dynamixel Protocol 2.0
        % This class is designed for using a Dynamixel XH430-W210, and an USB2DYNAMIXEL.
        % To use another Dynamixel model, such as PRO series, see their details in E-Manual(support.robotis.com) and edit below variables yourself.
        % Control table address
        ADDR_XH_TORQUE_ENABLE          = 64;                 % Control table address is different in Dynamixel model
        LEN_XH_TORQUE_ENABLE           = 1;
        % info. related to CURRENT
        % -648 ~ 648 * 2.69[mA]
        ADDR_XH_GOAL_CURRENT           = 102;
        ADDR_XH_PRESENT_CURRENT       = 126;
        LEN_XH_GOAL_CURRENT       = 2;
        LEN_XH_PRESENT_CURRENT    = 2;
        
        % info. related to POSITION
        % - 1,048,575 ~ 1,048,575 * 0.088 deg = -256[rev] ~ 256[rev]
        % 1[rev] : 0 ~ 4,095
        ADDR_XH_GOAL_POSITION           = 116;
        ADDR_XH_PRESENT_POSITION       = 132;
        LEN_XH_GOAL_POSITION       = 4;
        LEN_XH_PRESENT_POSITION    = 4;
        
        % The profile acceleration of the dynamixel;
        % This value could be set to 100;
        ADDR_XH_PROFILE_ACCELERATION = 108;
        LEN_XH_PROFILE_ACCELERATION = 4;
        
        % The profile velocity of the dynamixel
        % This value could be set to 112;
        ADDR_XH_PROFILE_VELOCITY = 112;
        LEN_XH_PROFILE_VELOCITY = 4;
        
        % Beware of that when trying to modify the operatiing mode, the
        % motors must be turned off.
        ADDR_XH_OPERATING_MODE         = 11;
        LEN_XH_OPERATING_MODE          = 1;
        
        % The PID parameters of Position Control Loop
        % Range: 0~16383 for all three paras
        % 
        ADDR_XH_KpD = 80;% KPD = KPD(TBL) / 16
        ADDR_XH_KpI = 82;% KPI = KPI(TBL) / 65536
        ADDR_XH_KpP = 84;% KPP = KPP(TBL) / 128
        
        LEN_XH_KpD = 2;
        LEN_XH_KpI = 2;
        LEN_XH_KpP = 2;
        
        
        
        OPERATING_MODE_CURRENT_MODE   = 0;
        OPERATING_MODE_POSITION_LIMITEDCURRENT = 5;
        
        % Protocol version
        PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel
        BAUDRATE                    = 1000000;
%         DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
        % ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
        
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_CURRENT_VALUE  = 0;
        DXL_MAXIMUM_CURRENT_VALUE  = 648;
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        
        ESC_CHARACTER               = 'e';          % Key for escaping loop
        
        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
        MAX_ID                      = 252;          % Maximum ID value
        
        
    end
    
    methods (Access = public)
        % e.g. comPort = 'COM3'
        % cableLengths_full: SIZE: numCmd x 1
        function interface = PoCaBotCASPRInterface(comPort, numCmd, ~,dynamixel_direction_reversed)
            interface@CableActuatorInterfaceBase();
            interface.comPort = comPort;
            interface.numCmd = numCmd;
            %interface.cableLengths_full = cableLengths_full;
            interface.dynamixel_direction_reversed = dynamixel_direction_reversed;
            if(dynamixel_direction_reversed)
                interface.dynamixel_direction_factor_position = -1;
                interface.dynamixel_direction_factor_current  = 1;
            else
                interface.dynamixel_direction_factor_position = 1;
                interface.dynamixel_direction_factor_current  =-1;
            end
            
            for i = 1: numCmd
                accessories_temp(i) = LargeMotorAccessories;
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
    
    methods (Access = private)
        % comPort = 'COM3'
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
            obj.port_num = portHandler(obj.comPort);
            
            % Initialize PacketHandler Structs
            packetHandler();
        end
    end
    
    methods
        function open(obj)
            % Initialize the serialpush port communication between Dynamixel and MATLAB
            % The input value is the COMPORT should be changed as per requirement
            % We ensure that the Dynamixel is also communicatiing with MATLAB at this
            % time. A predefined code on the arduino acknowledges this.
            if ~libisloaded(obj.lib_name)
                loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
            end
            
            if (openPort(obj.port_num))
                CASPR_log.Debug('Succeeded to open the port!\n');
            else
                unloadlibrary(obj.lib_name);
                CASPR_log.Error('Failed to open the port!\n');
            end
            
            % Set port baudrate
            if (setBaudRate(obj.port_num, obj.BAUDRATE))
                CASPR_log.Debug('Succeeded to change the baudrate!\n');
            else
                unloadlibrary(obj.lib_name);
                CASPR_log.Error('Failed to change the baudrate!\n');
            end
        end
        
        function close(obj)
            % Close port
            closePort(obj.port_num);
            
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
            obj.DXL_NUM = 0;
            obj.DXL_ID = [];
            success = 0;
            % Try to broadcast ping the Dynamixels
            broadcastPing(obj.port_num, obj.PROTOCOL_VERSION);
            for id = 0 : obj.MAX_ID
                if getBroadcastPingResult(obj.port_num, obj.PROTOCOL_VERSION, id)
                    obj.DXL_NUM = obj.DXL_NUM + 1;
                    obj.DXL_ID = [obj.DXL_ID;id];
                end
            end
            
            if(obj.numCmd ~= obj.DXL_NUM)
                obj.close();
                CASPR_log.Error('The number of the motors detected does not match the number you specified');
            end
            
            if(obj.DXL_NUM > 0)
                success = 1;
            end
        end
        
        % Method to send the vector of cable lengths (l_cmd) to hardware
        %
        % Argument l_cmd is a column vector with size (DXL_NUM x 1)
        %
        % Make sure that the dynamixels have been set to the extended
        % position mode or Current-based Position Control Mode
        % before invoking this function.
        function lengthCommandSend(obj, l_cmd)
            if(length(l_cmd) ~= obj.DXL_NUM)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument l0 and try again');
            end
            [dynamixel_position_cmd] = obj.len2anglecmd(l_cmd);
            obj.sync_write(obj.ADDR_XH_GOAL_POSITION, obj.LEN_XH_GOAL_POSITION, dynamixel_position_cmd);
        end
        
        % Method to send the initial cable length information to the
        % hardware interface and hardware (if required)
        %
        % For application of dynamixels, this funciton is used to specify
        % the initial length from the motor model outlet point to the
        % attachment point on the end effector.
        % The size of l0 here should be DXL_NUM X 1. The elements in l0 is
        % the absolute length, so they should be positive.
        function lengthInitialSend(obj, l0)
            if(length(l0) ~= obj.DXL_NUM)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument l0 and try again');
            end
            obj.cableLengths_initial = l0;
            
            deltaLengths_init = obj.cableLengths_full - obj.cableLengths_initial;
            for i = 1:obj.DXL_NUM
                obj.accessories(i).setInitState(deltaLengths_init(i));
            end
            [~, ret] = obj.sync_read(obj.ADDR_XH_PRESENT_POSITION, obj.LEN_XH_PRESENT_POSITION);
            obj.dynamixel_position_initial = ret;
        end
        
        function motorPositionCommandSend(obj, p_cmd)
            obj.sync_write(obj.ADDR_XH_GOAL_POSITION, obj.LEN_XH_GOAL_POSITION, p_cmd);
        end
        
        function [position] = motorPositionFeedbackRead(obj)
            [~, position] = obj.sync_read(obj.ADDR_XH_PRESENT_POSITION, obj.LEN_XH_PRESENT_POSITION);
        end
        
        % Method to read the cable lengths from the hardware (if available)
        function [length] = lengthFeedbackRead(obj)
            [bState, ret] = obj.sync_read(obj.ADDR_XH_PRESENT_POSITION, obj.LEN_XH_PRESENT_POSITION);
            if(bState)
                deltaAngle = (ret - obj.dynamixel_position_initial)/4096*2*pi;
                deltalength = zeros(size(deltaAngle));
                for i = 1:obj.DXL_NUM
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
            obj.toggleEnableAllDynamixel(obj.TORQUE_DISABLE);
        end
        
%         % Beware of that when trying to modify the operatiing mode, the
%         % motors must be turned off.
%         ADDR_XH_OPERATING_MODE         = 11;
%         LEN_XH_OPERATING_MODE          = 1;
%         
%         OPERATING_MODE_CURRENT_MODE   = 0;
%         OPERATING_MODE_POSITION_LIMITEDCURRENT = 5;
%         Argument operating_mode is OPERATING_MODE_CURRENT_MODE or
%         OPERATING_MODE_POSITION_LIMITEDCURRENT.
        function switchOperatingMode2CURRENT(obj)
            [~, mode] = obj.sync_read(obj.ADDR_XH_OPERATING_MODE, obj.LEN_XH_OPERATING_MODE);
            if(~all(mode == obj.OPERATING_MODE_CURRENT_MODE))
                obj.toggleEnableAllDynamixel(obj.TORQUE_DISABLE);
                obj.sync_write(obj.ADDR_XH_OPERATING_MODE, obj.LEN_XH_OPERATING_MODE, ones(obj.DXL_NUM,1)*obj.OPERATING_MODE_CURRENT_MODE);
            end
        end
        
        function switchOperatingMode2POSITION_LIMITEDCURRENT(obj)
            [~, mode] = obj.sync_read(obj.ADDR_XH_OPERATING_MODE, obj.LEN_XH_OPERATING_MODE);
            if(~all(mode == obj.OPERATING_MODE_POSITION_LIMITEDCURRENT))
                obj.toggleEnableAllDynamixel(obj.TORQUE_DISABLE);
                obj.sync_write(obj.ADDR_XH_OPERATING_MODE, obj.LEN_XH_OPERATING_MODE, ones(obj.DXL_NUM,1)*obj.OPERATING_MODE_POSITION_LIMITEDCURRENT);
            end
        end
        
        function [mode] = getOperatingMode(obj)
            [~, mode] = obj.sync_read(obj.ADDR_XH_OPERATING_MODE, obj.LEN_XH_OPERATING_MODE);
        end
        
        % Method to send the vector of current (c_cmd) to hardware
        %
        % Argument c_cmd is a column vector with size (DXL_NUM x 1)
        function currentCommandSend(obj, c_cmd)
            if(length(c_cmd) ~= obj.DXL_NUM)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument c_cmd and try again');
            end
            obj.sync_write(obj.ADDR_XH_GOAL_CURRENT, obj.LEN_XH_GOAL_CURRENT, c_cmd*obj.dynamixel_direction_factor_current);
        end
        
        % Method to read the current from the hardware (if available)
        function [current] = currentFeedbackRead(obj)
            [~, current] = obj.sync_read(obj.ADDR_XH_PRESENT_CURRENT, obj.LEN_XH_PRESENT_CURRENT);
            current = current*obj.dynamixel_direction_factor_current;
        end
        
        function setProfileAcceleration(obj,profile)
            obj.sync_write(obj.ADDR_XH_PROFILE_ACCELERATION, obj.LEN_XH_PROFILE_ACCELERATION, profile);
        end
        
        function setProfileVelocity(obj,profile)
            obj.sync_write(obj.ADDR_XH_PROFILE_VELOCITY, obj.LEN_XH_PROFILE_VELOCITY, profile);
        end
        
        % Tighten the cables without chaning the mode into CURRENT mode.
        % The precondition is that the motors are working under Extended
        % Position Mode, no matter with or without current constrained.
        function tightenCablesWithinPositionMode(obj)
            cmd = -1*obj.dynamixel_direction_reversed*1048574*ones(obj.DXL_NUM,1);
            obj.motorPositionCommandSend(cmd);
        end
        
        % Enable or Disable the motors
        function switchEnable(obj, cmd)
            obj.sync_write(obj.ADDR_XH_TORQUE_ENABLE, obj.LEN_XH_TORQUE_ENABLE, cmd);
        end
        
        % Set the PID paras of the Position loop
        function setKpD(obj,KpD)
            obj.KpD = KpD;
            obj.sync_write(obj.ADDR_XH_KpD, obj.LEN_XH_KpD, KpD);
        end
        function setKpI(obj,KpI)
            obj.KpI = KpI;
            obj.sync_write(obj.ADDR_XH_KpI, obj.LEN_XH_KpI, KpI);
        end
        function setKpP(obj,KpP)
            obj.KpP = KpP;
            obj.sync_write(obj.ADDR_XH_KpP, obj.LEN_XH_KpP, KpP);
        end
        % Get the PID parameters of the Position control loop
        function [KpD] = getKpD(obj)
            [~, KpD] = obj.sync_read(obj.ADDR_XH_KpD, obj.LEN_XH_KpD);
            obj.KpD = KpD;
        end
        function [KpI] = getKpI(obj)
            [~, KpI] = obj.sync_read(obj.ADDR_XH_KpI, obj.LEN_XH_KpI);
            obj.KpI = KpI;
        end
        function [KpP] = getKpP(obj)
            [~, KpP] = obj.sync_read(obj.ADDR_XH_KpP, obj.LEN_XH_KpP);
            obj.KpP = KpP;
        end
        
        % This function is used when the motor is working in position mode
        % with the KpI set to zero.
        % offset: the offset length of the cable caused by KpP.
        function [offset] = getCableOffsetByTensionByMotorAngleError(obj, tension)
            % when the supplied voltage is 12V, the stall torque from the
            % manual is 2.5N.m
            maxLoad = 2.5;%UNIT:N.m; This value of 1.5 N.m was obtained by our experiment.
            maxTension = maxLoad/obj.accessories(1).radius;
            current = tension/maxTension*648;
            Kp = obj.KpP/128;% KPP = KPP(TBL) / 128
            errAngle = current./Kp;
            offset = -1 * errAngle/4096*obj.accessories(1).len_per_circle;
        end
    end
    
    methods (Access = private)
        % This function has built in the math model of the spool and the
        % dynamixel holder. the input argument relLen is a column vector
        % with size DXL_NUM X 1, specifying the absolute length we need
        % to achieve by pulling or loosing the cables.
        function [dynamixel_position_cmd] = len2anglecmd(obj, relLen)
            if(length(relLen) ~= obj.DXL_NUM)
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument relLen and try again');
            end
            delta_lengths = relLen - obj.cableLengths_initial;
            for i = 1:obj.DXL_NUM
                angle_delta(i) = obj.accessories(i).getDeltaAngle(delta_lengths(i));
            end
            dynamixel_position_delta = obj.dynamixel_direction_factor_position * angle_delta/2/pi*4096;
            dynamixel_position_cmd = obj.dynamixel_position_initial + dynamixel_position_delta';
        end
        
        function [pos] = radian2dynamixelposition(obj, angle)
            if(~isvector(angle))
                obj.close();
                CASPR_log.Error('Input argument error, please check the size of the argument angle_delta and try again');
            end
            pos = angle/2/pi*4096;
        end
        
        % cmd could be TORQUE_ENABLE or TORQUE_DISABLE
        function toggleEnableSingleDynamixel(obj, ID, cmd)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ID, obj.ADDR_XH_TORQUE_ENABLE, cmd);
            if( getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) == 0 )
                CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully controlled \n', ID));
            end
        end
        
        % cmd could be TORQUE_ENABLE or TORQUE_DISABLE
        function toggleEnableAllDynamixel(obj, cmd)
            for dxl = 1:obj.DXL_NUM
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(dxl), obj.ADDR_XH_TORQUE_ENABLE, cmd);
                if( getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) == 0 )
                    CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully controlled \n', obj.DXL_ID(dxl)));
                end
            end
        end
        
        % write into a single motor
        function single_write(obj, dxl_ID, start_address, data_length, data)
            switch data_length
                case 1
                    write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, dxl_ID, start_address, typecast(int8(data), 'uint8'));
                    if( getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) == 0 )
                        CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully written in \n', dxl_ID));
                    end
                case 2
                    write2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, dxl_ID, start_address, typecast(int16(data), 'uint16'));
                    if( getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) == 0 )
                        CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully written in \n', dxl_ID));
                    end
                case 4
                    write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, dxl_ID, start_address, typecast(int32(data), 'uint32'));
                    if( getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION) == obj.COMM_SUCCESS && getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION) == 0 )
                        CASPR_log.Debug(sprintf('Dynamixel #%d has been successfully written in \n', dxl_ID));
                    end
                otherwise
                    obj.close();
                    CASPR_log.Error('Please input applicable data length!');
            end
        end
        
        % read from a single motor
        function [ret] = single_read(obj, dxl_ID, start_address, data_length)
            switch data_length
                case 1
                    rep = read1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, dxl_ID, start_address);
                    ret = typecast(uint8(rep), 'int8');
                case 2
                    rep = read2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, dxl_ID, start_address);
                    ret = typecast(uint16(rep), 'int16');
                case 4
                    rep = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, dxl_ID, start_address);
                    ret = typecast(uint32(rep), 'int32');
                otherwise
                    obj.close();
                    CASPR_log.Error('Please input applicable data length!');
            end
        end
        
        % argument data_input is a column vector with size (DXL_NUM x 1) and the size of each element
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
            
            % Initialize Groupsyncwrite Structs
            groupwrite_num = groupSyncWrite(obj.port_num, obj.PROTOCOL_VERSION, start_address, data_length);
            for dxl = 1:obj.DXL_NUM
                % Add Dynamixel goal current value to the Syncwrite storage
                dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, obj.DXL_ID(dxl), data(dxl), data_length);
                if dxl_addparam_result ~= true
                    obj.close();
                    CASPR_log.Error(sprintf('[ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID(dxl)));
                end
            end
            
            
            groupSyncWriteTxPacket(groupwrite_num);
            
            % Clear syncwrite parameter storage
            groupSyncWriteClearParam(groupwrite_num);
        end
        
        % bState = true means successful
        function [bState, ret] = sync_read(obj, start_address, data_length)
            ret = zeros(obj.DXL_NUM,1);
            
            % Prepare the parameters
            groupread_num = groupSyncRead(obj.port_num, obj.PROTOCOL_VERSION, start_address, data_length);
            for dxl = 1:obj.DXL_NUM
                dxl_addparam_result = groupSyncReadAddParam(groupread_num, obj.DXL_ID(dxl));
                if dxl_addparam_result ~= true
                    obj.close();
                    CASPR_log.Error(sprintf('[ID:%03d] groupSyncRead addparam failed', obj.DXL_ID(dxl)));
                end
            end
            groupSyncReadTxRxPacket(groupread_num);
            
            bState = true;
            for dxl = 1:obj.DXL_NUM
                % Check if groupsyncread data of Dynamixels is available
                dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, obj.DXL_ID(dxl), start_address, data_length);
                if dxl_getdata_result ~= true
                    CASPR_log.Warn(sprintf('[ID:%03d] groupSyncRead getdata failed', obj.DXL_ID(dxl)));
                    bState = false;
                    break;
                else
                    rep = groupSyncReadGetData(groupread_num, obj.DXL_ID(dxl), start_address, data_length);
                    switch data_length
                        case 1
                            ret(dxl) = typecast(uint8(rep), 'int8');
                        case 2
                            ret(dxl) = typecast(uint16(rep), 'int16');
                        case 4
                            ret(dxl) = typecast(uint32(rep), 'int32');
                        otherwise
                            obj.close();
                            CASPR_log.Error('Please input applicable data length!');
                    end
                end
            end
            groupSyncReadClearParam(groupread_num);
        end
    end
end