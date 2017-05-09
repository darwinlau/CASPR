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
% *********     Bulk Read Example      *********
%
%
% Available Dynamixel model on this example : MX or X series set to Protocol 1.0
% This example is designed for using two Dynamixel MX-28, and an USB2DYNAMIXEL.
% To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below variables yourself.
% Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 1000000)
%

clc;
clear all;

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
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLE           = 24;           % Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION           = 30;
ADDR_MX_PRESENT_POSITION        = 36;
ADDR_MX_MOVING                  = 46;

% Data Byte Length
LEN_MX_GOAL_POSITION            = 2;
LEN_MX_PRESENT_POSITION         = 2;
LEN_MX_MOVING                   = 1;

% Protocol version
PROTOCOL_VERSION                = 1.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL1_ID                         = 1;            % Dynamixel#1 ID: 1
DXL2_ID                         = 2;            % Dynamixel#2 ID: 2
BAUDRATE                        = 1000000;
DEVICENAME                      = 'COM1';       % Check which port is being used on your controller
                                                % ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE                   = 1;            % Value for enabling the torque
TORQUE_DISABLE                  = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE      = 100;          % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE      = 4000;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD     = 10;           % Dynamixel moving status threshold

ESC_CHARACTER                   = 'e';          % Key for escaping loop

COMM_SUCCESS                    = 0;            % Communication Success result value
COMM_TX_FAIL                    = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupbulkread Structs
group_num = groupBulkRead(port_num, PROTOCOL_VERSION);

index = 1;
dxl_comm_result = COMM_TX_FAIL;                 % Communication result
dxl_addparam_result = false;                    % AddParam result
dxl_getdata_result = false;                     % GetParam result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                                  % Dynamixel error
dxl1_present_position = 0;                      % Present position
dxl2_moving = 0;                                % Dynamixel moving status

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


% Enable Dynamixel#1 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Enable Dynamixel#2 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupBulkReadAddParam(group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL1_ID);
    return;
end

% Add parameter storage for Dynamixel#2 present moving value
dxl_addparam_result = groupBulkReadAddParam(group_num, DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL2_ID);
    return;
end


while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Write Dynamixel#1 goal position
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position(index));
    if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
    elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
        printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
    end

    % Write Dynamixel#2 goal position
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position(index));
    if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
    elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
        printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
    end

    while 1
        % Bulkread present position and moving status
        groupBulkReadTxRxPacket(group_num);
        if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
            printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
        end

        % Check if groupbulkread data of Dynamixel#1 is available
        dxl_getdata_result = groupBulkReadIsAvailable(group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL1_ID);
            return;
        end

        % Check if groupbulkread data of Dynamixel#2 is available
        dxl_getdata_result = groupBulkReadIsAvailable(group_num, DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL2_ID);
            return;
        end

        % Get Dynamixel#1 present position value
        dxl1_present_position = groupBulkReadGetData(group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

        % Get Dynamixel#2 moving status value
        dxl2_moving = groupBulkReadGetData(group_num, DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING);

        fprintf('[ID:%03d] Present Position : %d \t [ID:%03d] Is Moving : %d\n', DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_moving);

        if ~(abs(dxl_goal_position(index) - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD);
            break;
        end
    end

    % Change goal position
    if (index == 1)
      index = 2;
    else
      index = 1;
    end
end


% Disable Dynamixel#1 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
end

% Disable Dynamixel#2 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
