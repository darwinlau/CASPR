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
% protocol_combined.c
%
%  Created on: 2016. 6. 7.
%      Author: Ryu Woon Jung (Leon)
%

%
% *********     Protocol Combined Example      *********
%
%
% Available Dynamixel model on this example : All models using Protocol 1.0 and 2.0
% This example is tested with a Dynamixel MX-28, a Dynamixel PRO 54-200 and an USB2DYNAMIXEL
% Be sure that properties of Dynamixel MX and PRO are already set as %% MX - ID : 1 / Baudnum : 1 (Baudrate : 1000000) , PRO - ID : 1 / Baudnum : 3 (Baudrate : 1000000)
%

% Be aware that:
% This example configures two different control tables (especially, if it uses Dynamixel and Dynamixel PRO). It may modify critical Dynamixel parameter on the control table, if Dynamixels have wrong ID.
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
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Control table address for Dynamixel MX
ADDR_MX_TORQUE_ENABLE           = 24;             % Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION           = 30;
ADDR_MX_PRESENT_POSITION        = 36;

% Control table address for Dynamixel PRO
ADDR_PRO_TORQUE_ENABLE          = 562;
ADDR_PRO_GOAL_POSITION          = 596;
ADDR_PRO_PRESENT_POSITION       = 611;

% Protocol version
PROTOCOL_VERSION1               = 1.0;            % See which protocol version is used in the Dynamixel
PROTOCOL_VERSION2               = 2.0;

% Default setting
DXL1_ID                         = 1;              % Dynamixel#1 ID: 1
DXL2_ID                         = 2;              % Dynamixel#2 ID: 2
BAUDRATE                        = 1000000;
DEVICENAME                      = 'COM1';         % Check which port is being used on your controller
                                                  % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0'

TORQUE_ENABLE                   = 1;              % Value for enabling the torque
TORQUE_DISABLE                  = 0;              % Value for disabling the torque
DXL1_MINIMUM_POSITION_VALUE     = 100;            % Dynamixel will rotate between this value
DXL1_MAXIMUM_POSITION_VALUE     = 4000;           % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL2_MINIMUM_POSITION_VALUE     = -150000;
DXL2_MAXIMUM_POSITION_VALUE     = 150000;
DXL1_MOVING_STATUS_THRESHOLD    = 10;             % Dynamixel MX moving status threshold
DXL2_MOVING_STATUS_THRESHOLD    = 20;             % Dynamixel PRO moving status threshold

ESC_CHARACTER                   = 'e';            % Key for escaping loop

COMM_SUCCESS                    = 0;              % Communication Success result value
COMM_TX_FAIL                    = -1001;          % Communication Tx Failed


% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;                   % Communication result
dxl1_goal_position = [DXL1_MINIMUM_POSITION_VALUE DXL1_MAXIMUM_POSITION_VALUE];         % Goal position of Dynamixel MX
dxl2_goal_position = [DXL2_MINIMUM_POSITION_VALUE DXL2_MAXIMUM_POSITION_VALUE];         % Goal position of Dynamixel PRO

dxl_error = 0;                                    % Dynamixel error
dxl1_present_position = 0;                        % Present position of Dynamixel MX
dxl2_present_position = 0;                        % Present position of Dynamixel PRO

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


% Enable Dynamixel#1 torque
write1ByteTxRx(port_num, PROTOCOL_VERSION1, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION1) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION1, getLastTxRxResult(port_num, PROTOCOL_VERSION1));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION1) ~= 0
    printRxPacketError(PROTOCOL_VERSION1, getLastRxPacketError(port_num, PROTOCOL_VERSION1));
else
    fprintf('Dynamixel #%d has been successfully connected \n', DXL1_ID);
end

% Enable Dynamixel#2 torque
write1ByteTxRx(port_num, PROTOCOL_VERSION2, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION2) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION2, getLastTxRxResult(port_num, PROTOCOL_VERSION2));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION2) ~= 0
    printRxPacketError(PROTOCOL_VERSION2, getLastRxPacketError(port_num, PROTOCOL_VERSION2));
else
    fprintf('Dynamixel #%d has been successfully connected \n', DXL2_ID);
end


while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Write Dynamixel#1 goal position
    write2ByteTxRx(port_num, PROTOCOL_VERSION1, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_position(index));
    if getLastTxRxResult(port_num, PROTOCOL_VERSION1) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION1, getLastTxRxResult(port_num, PROTOCOL_VERSION1));
    elseif getLastRxPacketError(port_num, PROTOCOL_VERSION1) ~= 0
        printRxPacketError(PROTOCOL_VERSION1, getLastRxPacketError(port_num, PROTOCOL_VERSION1));
    end

    % Write Dynamixel#2 goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION2, DXL2_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl2_goal_position(index)), 'uint32'));
    if getLastTxRxResult(port_num, PROTOCOL_VERSION2) ~= COMM_SUCCESS
        printTxRxResult(PROTOCOL_VERSION2, getLastTxRxResult(port_num, PROTOCOL_VERSION2));
    elseif getLastRxPacketError(port_num, PROTOCOL_VERSION2) ~= 0
        printRxPacketError(PROTOCOL_VERSION2, getLastRxPacketError(port_num, PROTOCOL_VERSION2));
    end

    while 1
      % Read Dynamixel#1 present position
    	dxl1_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION1, DXL1_ID, ADDR_MX_PRESENT_POSITION);
      if getLastTxRxResult(port_num, PROTOCOL_VERSION1) ~= COMM_SUCCESS
          printTxRxResult(PROTOCOL_VERSION1, getLastTxRxResult(port_num, PROTOCOL_VERSION1));
      elseif getLastRxPacketError(port_num, PROTOCOL_VERSION1) ~= 0
          printRxPacketError(PROTOCOL_VERSION1, getLastRxPacketError(port_num, PROTOCOL_VERSION1));
      end

      % Read Dynamixel#2 present position
      dxl2_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION2, DXL2_ID, ADDR_PRO_PRESENT_POSITION);
      if getLastTxRxResult(port_num, PROTOCOL_VERSION2) ~= COMM_SUCCESS
          printTxRxResult(PROTOCOL_VERSION2, getLastTxRxResult(port_num, PROTOCOL_VERSION2));
      elseif getLastRxPacketError(port_num, PROTOCOL_VERSION2) ~= 0
          printRxPacketError(PROTOCOL_VERSION2, getLastRxPacketError(port_num, PROTOCOL_VERSION2));
      end

      fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d [ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL1_ID, dxl1_goal_position(index), dxl1_present_position, DXL2_ID, dxl2_goal_position(index), typecast(uint32(dxl2_present_position), 'int32'));

      if ~((abs(dxl1_goal_position(index) - dxl1_present_position) > DXL1_MOVING_STATUS_THRESHOLD) || (abs(dxl2_goal_position(index) - typecast(uint32(dxl2_present_position), 'int32')) > DXL2_MOVING_STATUS_THRESHOLD));
          break;
      end
    end

    % Change goal position
    if index == 1
        index = 2;
    else
        index = 1;
    end
end

  % Disable Dynamixel#1 Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION1, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION1) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION1, getLastTxRxResult(port_num, PROTOCOL_VERSION1));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION1) ~= 0
    printRxPacketError(PROTOCOL_VERSION1, getLastRxPacketError(port_num, PROTOCOL_VERSION1));
end

  % Disable Dynamixel#2 Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION2, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION2) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION2, getLastTxRxResult(port_num, PROTOCOL_VERSION2));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION2) ~= 0
    printRxPacketError(PROTOCOL_VERSION2, getLastRxPacketError(port_num, PROTOCOL_VERSION2));
end


% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
