%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2016, ROBOTIS CO., LTD.
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% * Redistributions of source code must retain the above copyright notice, this
%   list of conditions and the following disclaimer.
%
% * Redistributions in binary form must reproduce the above copyright notice,
%   this list of conditions and the following disclaimer in the documentation
%   and/or other materials provided with the distribution.
%
% * Neither the name of ROBOTIS nor the names of its
%   contributors may be used to endorse or promote products derived from
%   this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author: Ryu Woon Jung (Leon)

%
% *********     Sync Read and Sync Write Example      *********
%
%
% Available Dynamixel model on this example : All models using Protocol 2.0
% This example is designed for using two Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
% To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below variables yourself.
% Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000 [1M])
%

clc;
clear all;
isDebugMode = 1;
lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end

% Control table address
ADDR_XH_TORQUE_ENABLE          = 64;                 % Control table address is different in Dynamixel model
ADDR_XH_GOAL_CURRENT           = 102;
ADDR_XH_PRESENT_CURRENT       = 126;

% Data Byte Length
LEN_XH_GOAL_CURRENT       = 2;
LEN_XH_PRESENT_CURRENT    = 2;

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_NUM                     = 4;
DXL_ID                      = zeros(DXL_NUM,1);
DXL_ID(1)                   = 0;            % Dynamixel#0 ID: 0
DXL_ID(2)                   = 1;            % Dynamixel#1 ID: 1
DXL_ID(3)                   = 2;            % Dynamixel#2 ID: 2
DXL_ID(4)                   = 3;            % Dynamixel#3 ID: 3
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
% ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_CURRENT_VALUE  = 0;
DXL_MAXIMUM_CURRENT_VALUE  = 648;
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupsyncwrite Structs
groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_XH_GOAL_CURRENT, LEN_XH_GOAL_CURRENT);
% Initialize Groupsyncread Structs
groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_XH_PRESENT_CURRENT, LEN_XH_PRESENT_CURRENT);

index = 1;

dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_addparam_result = false;              % AddParam result
dxl_getdata_result = false;               % GetParam result
%dxl_goal_current = [DXL_MINIMUM_CURRENT_VALUE DXL_MAXIMUM_CURRENT_VALUE];         % Goal position
dxl_goal_current = round(0:0.3:100);
% for dxl = 1:DXL_NUM
%     dxl_goal_current(dxl) = 10;
% end
dxl_error = 0;                              % Dynamixel error
dxl_present_current = zeros(DXL_NUM);


% Open port
if (openPort(port_num))
    if(isDebugMode)
        fprintf('Succeeded to open the port!\n');
    end
else
    unloadlibrary(lib_name);
    if(isDebugMode)
        fprintf('Failed to open the port!\n');
    end
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    if(isDebugMode)
        fprintf('Succeeded to change the baudrate!\n');
    end
else
    unloadlibrary(lib_name);
    if(isDebugMode)
        fprintf('Failed to change the baudrate!\n');
    end
    input('Press any key to terminate...\n');
    return;
end



for dxl = 1:DXL_NUM
    % Enable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(dxl), ADDR_XH_TORQUE_ENABLE, TORQUE_ENABLE);
    %     if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
    %         printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
    %     elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    %         printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
    %     else
    %         fprintf('Dynamixel #%d has been successfully connected \n', DXL_ID(dxl));
    %     end
    if( getLastTxRxResult(port_num, PROTOCOL_VERSION) == COMM_SUCCESS && getLastRxPacketError(port_num, PROTOCOL_VERSION) == 0 )
        if(isDebugMode)
            fprintf('Dynamixel #%d has been successfully connected \n', DXL_ID(dxl));
        end
    end
    % Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_ID(dxl));
    if dxl_addparam_result ~= true
        if(isDebugMode)
            fprintf('[ID:%03d] groupSyncRead addparam failed', DXL_ID(dxl));
        end
        return;
    end
end

while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end
    while 1
        tic
        for dxl = 1:DXL_NUM
            % Add Dynamixel goal current value to the Syncwrite storage
            dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_ID(dxl), dxl_goal_current(index), LEN_XH_GOAL_CURRENT);
            if dxl_addparam_result ~= true
                if(isDebugMode)
                    fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL_ID(dxl));
                end
                break;
            end
        end
        
        % Syncwrite goal position
        groupSyncWriteTxPacket(groupwrite_num);
        %         if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
        %             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
        %         end
        
        % Clear syncwrite parameter storage
        groupSyncWriteClearParam(groupwrite_num);
        
        % Syncread present position
        groupSyncReadTxRxPacket(groupread_num);
        %         if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
        %             printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
        %         end
        
        for dxl = 1:DXL_NUM
            % Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL_ID(dxl), ADDR_XH_PRESENT_CURRENT, LEN_XH_PRESENT_CURRENT);
            if dxl_getdata_result ~= true
                if(isDebugMode)
                    fprintf('[ID:%03d] groupSyncRead getdata failed', DXL_ID(dxl));
                end
                break;
            end
            
            % Get Dynamixel#1 present position value
            dxl_present_current(dxl) = groupSyncReadGetData(groupread_num, DXL_ID(dxl), ADDR_XH_PRESENT_CURRENT, LEN_XH_PRESENT_CURRENT);
            if(isDebugMode)
                fprintf('[ID:%03d] G:%03d  A:%03d\t', DXL_ID(dxl), dxl_goal_current(index),dxl_present_current(dxl));
            end
        end
        if(isDebugMode)
            fprintf('\n');
        end
        %         if ~((abs(dxl_goal_current(index) - typecast(uint32(dxl1_present_current), 'int32')) > DXL_MOVING_STATUS_THRESHOLD) ||
        %              (abs(dxl_goal_current(index) - typecast(uint32(dxl2_present_current), 'int32')) > DXL_MOVING_STATUS_THRESHOLD))
        %             break;
        %         end
        toc
        % Change goal current
        if index == 300
            index = 1;
        else
            index = index + 1;
        end
    end
end

for dxl = 1:DXL_NUM
    % Disable Dynamixel#1 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(dxl), ADDR_XH_TORQUE_ENABLE, TORQUE_DISABLE);
    if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
    elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
        printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
    end
end


% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
