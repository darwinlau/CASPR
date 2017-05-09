/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_C_H_


#include "packet_handler.h"

WINDECLSPEC void        printTxRxResult1    (int result);
WINDECLSPEC void        printRxPacketError1 (uint8_t error);

WINDECLSPEC int         getLastTxRxResult1  (int port_num);
WINDECLSPEC uint8_t     getLastRxPacketError1   (int port_num);

WINDECLSPEC void        setDataWrite1       (int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data);
WINDECLSPEC uint32_t    getDataRead1        (int port_num, uint16_t data_length, uint16_t data_pos);

WINDECLSPEC void        txPacket1           (int port_num);
WINDECLSPEC void        rxPacket1           (int port_num);
WINDECLSPEC void        txRxPacket1         (int port_num);

WINDECLSPEC void        ping1               (int port_num, uint8_t id);
WINDECLSPEC uint16_t    pingGetModelNum1    (int port_num, uint8_t id);

// broadcastPing
WINDECLSPEC void        broadcastPing1      (int port_num);
WINDECLSPEC uint8_t     getBroadcastPingResult1 (int port_num, int id);

WINDECLSPEC void        action1             (int port_num, uint8_t id);
WINDECLSPEC void        reboot1             (int port_num, uint8_t id);
WINDECLSPEC void        factoryReset1       (int port_num, uint8_t id, uint8_t option);

WINDECLSPEC void        readTx1             (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        readRx1             (int port_num, uint16_t length);
WINDECLSPEC void        readTxRx1           (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        read1ByteTx1        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint8_t     read1ByteRx1        (int port_num);
WINDECLSPEC uint8_t     read1ByteTxRx1      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        read2ByteTx1        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint16_t    read2ByteRx1        (int port_num);
WINDECLSPEC uint16_t    read2ByteTxRx1      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        read4ByteTx1        (int port_num, uint8_t id, uint16_t address);
WINDECLSPEC uint32_t    read4ByteRx1        (int port_num);
WINDECLSPEC uint32_t    read4ByteTxRx1      (int port_num, uint8_t id, uint16_t address);

WINDECLSPEC void        writeTxOnly1        (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        writeTxRx1          (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        write1ByteTxOnly1   (int port_num, uint8_t id, uint16_t address, uint8_t data);
WINDECLSPEC void        write1ByteTxRx1     (int port_num, uint8_t id, uint16_t address, uint8_t data);

WINDECLSPEC void        write2ByteTxOnly1   (int port_num, uint8_t id, uint16_t address, uint16_t data);
WINDECLSPEC void        write2ByteTxRx1     (int port_num, uint8_t id, uint16_t address, uint16_t data);

WINDECLSPEC void        write4ByteTxOnly1   (int port_num, uint8_t id, uint16_t address, uint32_t data);
WINDECLSPEC void        write4ByteTxRx1     (int port_num, uint8_t id, uint16_t address, uint32_t data);

WINDECLSPEC void        regWriteTxOnly1     (int port_num, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        regWriteTxRx1       (int port_num, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        syncReadTx1         (int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
WINDECLSPEC void        syncWriteTxOnly1    (int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);

// param : LEN1 ID1 ADDR1 LEN2 ID2 ADDR2 ...
WINDECLSPEC void        bulkReadTx1         (int port_num, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
WINDECLSPEC void        bulkWriteTxOnly1    (int port_num, uint16_t param_length);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_C_H_ */
