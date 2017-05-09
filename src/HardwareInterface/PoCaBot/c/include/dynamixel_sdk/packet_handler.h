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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_C_H_


#include <stdio.h>
#include "robotis_def.h"
#include "port_handler.h"

#define BROADCAST_ID        0xFE    // 254
#define MAX_ID              0xFC    // 252

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

/* Instruction for DXL Protocol */
#define INST_PING           1
#define INST_READ           2
#define INST_WRITE          3
#define INST_REG_WRITE      4
#define INST_ACTION         5
#define INST_FACTORY_RESET  6
#define INST_SYNC_WRITE     131     // 0x83
#define INST_BULK_READ      146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT         8
#define INST_STATUS         85      // 0x55
#define INST_SYNC_READ      130     // 0x82
#define INST_BULK_WRITE     147     // 0x93

// Communication Result
#define COMM_SUCCESS        0       // tx or rx packet communication success
#define COMM_PORT_BUSY      -1000   // Port is busy (in use)
#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        -1002   // Failed get status packet
#define COMM_TX_ERROR       -2000   // Incorrect instruction packet
#define COMM_RX_WAITING     -3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     -3001   // There is no status packet
#define COMM_RX_CORRUPT     -3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9000   //

typedef struct
{
  uint8_t     *data_write;
  uint8_t     *data_read;
  uint8_t     *tx_packet;
  uint8_t     *rx_packet;
  uint8_t     error;
  int         communication_result;
  uint8_t     *broadcast_ping_id_list;
}PacketData;

PacketData *packetData;

WINDECLSPEC void        packetHandler       ();

WINDECLSPEC void        printTxRxResult     (int protocol_version, int result);
WINDECLSPEC void        printRxPacketError  (int protocol_version, uint8_t error);

WINDECLSPEC int         getLastTxRxResult   (int port_num, int protocol_version);
WINDECLSPEC uint8_t     getLastRxPacketError    (int port_num, int protocol_version);

WINDECLSPEC void        setDataWrite        (int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos, uint32_t data);
WINDECLSPEC uint32_t    getDataRead         (int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos);

WINDECLSPEC void        txPacket            (int port_num, int protocol_version);

WINDECLSPEC void        rxPacket            (int port_num, int protocol_version);

WINDECLSPEC void        txRxPacket          (int port_num, int protocol_version);

WINDECLSPEC void        ping                (int port_num, int protocol_version, uint8_t id);

WINDECLSPEC uint16_t    pingGetModelNum     (int port_num, int protocol_version, uint8_t id);

// broadcastPing
WINDECLSPEC void        broadcastPing       (int port_num, int protocol_version);
WINDECLSPEC uint8_t     getBroadcastPingResult  (int port_num, int protocol_version, int id);

WINDECLSPEC void        reboot              (int port_num, int protocol_version, uint8_t id);

WINDECLSPEC void        factoryReset        (int port_num, int protocol_version, uint8_t id, uint8_t option);

WINDECLSPEC void        readTx              (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void        readRx              (int port_num, int protocol_version, uint16_t length);
WINDECLSPEC void        readTxRx            (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void        read1ByteTx         (int port_num, int protocol_version, uint8_t id, uint16_t address);
WINDECLSPEC uint8_t     read1ByteRx         (int port_num, int protocol_version);
WINDECLSPEC uint8_t     read1ByteTxRx       (int port_num, int protocol_version, uint8_t id, uint16_t address);

WINDECLSPEC void        read2ByteTx         (int port_num, int protocol_version, uint8_t id, uint16_t address);
WINDECLSPEC uint16_t    read2ByteRx         (int port_num, int protocol_version);
WINDECLSPEC uint16_t    read2ByteTxRx       (int port_num, int protocol_version, uint8_t id, uint16_t address);

WINDECLSPEC void        read4ByteTx         (int port_num, int protocol_version, uint8_t id, uint16_t address);
WINDECLSPEC uint32_t    read4ByteRx         (int port_num, int protocol_version);
WINDECLSPEC uint32_t    read4ByteTxRx       (int port_num, int protocol_version, uint8_t id, uint16_t address);

WINDECLSPEC void    writeTxOnly             (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void    writeTxRx               (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void    write1ByteTxOnly        (int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data);
WINDECLSPEC void    write1ByteTxRx          (int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data);

WINDECLSPEC void    write2ByteTxOnly        (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data);
WINDECLSPEC void    write2ByteTxRx          (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data);

WINDECLSPEC void    write4ByteTxOnly        (int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data);
WINDECLSPEC void    write4ByteTxRx          (int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data);

WINDECLSPEC void    regWriteTxOnly          (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
WINDECLSPEC void    regWriteTxRx            (int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

WINDECLSPEC void    syncReadTx              (int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

WINDECLSPEC void    syncWriteTxOnly         (int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length);

WINDECLSPEC void    bulkReadTx              (int port_num, int protocol_version, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

WINDECLSPEC void    bulkWriteTxOnly         (int port_num, int protocol_version, uint16_t param_length);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_C_H_ */
