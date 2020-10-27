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

/* Author: zerom, Ryu Woon Jung (Leon) */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_


#include <stdio.h>
#include <vector>
#include "dynamixel_sdk/port_handler.h"

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
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT             8
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

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

namespace dynamixel
{

class WINDECLSPEC PacketHandler
{
 protected:
  PacketHandler() { }

 public:
  static PacketHandler *getPacketHandler(float protocol_version = 2.0);

  virtual ~PacketHandler() { }

  virtual float   getProtocolVersion() = 0;

  virtual void    printTxRxResult(int result) = 0;
  virtual void    printRxPacketError(uint8_t error) = 0;

  virtual int txPacket        (PortHandler *port, uint8_t *txpacket) = 0;
  virtual int rxPacket        (PortHandler *port, uint8_t *rxpacket) = 0;
  virtual int txRxPacket      (PortHandler *port, uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error = 0) = 0;

  virtual int ping            (PortHandler *port, uint8_t id, uint8_t *error = 0) = 0;
  virtual int ping            (PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error = 0) = 0;

  // broadcastPing
  virtual int broadcastPing   (PortHandler *port, std::vector<uint8_t> &id_list) = 0;

  virtual int action          (PortHandler *port, uint8_t id) = 0;
  virtual int reboot          (PortHandler *port, uint8_t id, uint8_t *error = 0) = 0;
  virtual int factoryReset    (PortHandler *port, uint8_t id, uint8_t option = 0, uint8_t *error = 0) = 0;


  virtual int readTx          (PortHandler *port, uint8_t id, uint16_t address, uint16_t length) = 0;
  virtual int readRx          (PortHandler *port, uint8_t id, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;
  virtual int readTxRx        (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

  virtual int read1ByteTx     (PortHandler *port, uint8_t id, uint16_t address) = 0;
  virtual int read1ByteRx     (PortHandler *port, uint8_t id, uint8_t *data, uint8_t *error = 0) = 0;
  virtual int read1ByteTxRx   (PortHandler *port, uint8_t id, uint16_t address, uint8_t *data, uint8_t *error = 0) = 0;

  virtual int read2ByteTx     (PortHandler *port, uint8_t id, uint16_t address) = 0;
  virtual int read2ByteRx     (PortHandler *port, uint8_t id, uint16_t *data, uint8_t *error = 0) = 0;
  virtual int read2ByteTxRx   (PortHandler *port, uint8_t id, uint16_t address, uint16_t *data, uint8_t *error = 0) = 0;

  virtual int read4ByteTx     (PortHandler *port, uint8_t id, uint16_t address) = 0;
  virtual int read4ByteRx     (PortHandler *port, uint8_t id, uint32_t *data, uint8_t *error = 0) = 0;
  virtual int read4ByteTxRx   (PortHandler *port, uint8_t id, uint16_t address, uint32_t *data, uint8_t *error = 0) = 0;

  virtual int writeTxOnly     (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) = 0;
  virtual int writeTxRx       (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

  virtual int write1ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint8_t data) = 0;
  virtual int write1ByteTxRx  (PortHandler *port, uint8_t id, uint16_t address, uint8_t data, uint8_t *error = 0) = 0;

  virtual int write2ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t data) = 0;
  virtual int write2ByteTxRx  (PortHandler *port, uint8_t id, uint16_t address, uint16_t data, uint8_t *error = 0) = 0;

  virtual int write4ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint32_t data) = 0;
  virtual int write4ByteTxRx  (PortHandler *port, uint8_t id, uint16_t address, uint32_t data, uint8_t *error = 0) = 0;

  virtual int regWriteTxOnly  (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) = 0;
  virtual int regWriteTxRx    (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0) = 0;

  virtual int syncReadTx      (PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length) = 0;
  // SyncReadRx   -> GroupSyncRead class
  // SyncReadTxRx -> GroupSyncRead class

  virtual int syncWriteTxOnly (PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length) = 0;

  virtual int bulkReadTx      (PortHandler *port, uint8_t *param, uint16_t param_length) = 0;
  // BulkReadRx   -> GroupBulkRead class
  // BulkReadTxRx -> GroupBulkRead class

  virtual int bulkWriteTxOnly (PortHandler *port, uint8_t *param, uint16_t param_length) = 0;
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PACKETHANDLER_H_ */
