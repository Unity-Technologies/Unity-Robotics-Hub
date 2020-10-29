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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_


#include "dynamixel_sdk/packet_handler.h"

namespace dynamixel
{

class WINDECLSPEC Protocol1PacketHandler : public PacketHandler
{
 private:
  static Protocol1PacketHandler *unique_instance_;

  Protocol1PacketHandler();

 public:
  static Protocol1PacketHandler *getInstance() { return unique_instance_; }

  virtual ~Protocol1PacketHandler() { }

  float   getProtocolVersion() { return 1.0; }

  void    printTxRxResult(int result);
  void    printRxPacketError(uint8_t error);

  int txPacket        (PortHandler *port, uint8_t *txpacket);
  int rxPacket        (PortHandler *port, uint8_t *rxpacket);
  int txRxPacket      (PortHandler *port, uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error = 0);

  int ping            (PortHandler *port, uint8_t id, uint8_t *error = 0);
  int ping            (PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error = 0);

  // broadcastPing
  int broadcastPing   (PortHandler *port, std::vector<uint8_t> &id_list);

  int action          (PortHandler *port, uint8_t id);
  int reboot          (PortHandler *port, uint8_t id, uint8_t *error = 0);
  int factoryReset    (PortHandler *port, uint8_t id, uint8_t option, uint8_t *error = 0);


  int readTx          (PortHandler *port, uint8_t id, uint16_t address, uint16_t length);
  int readRx          (PortHandler *port, uint8_t id, uint16_t length, uint8_t *data, uint8_t *error = 0);
  int readTxRx        (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);

  int read1ByteTx     (PortHandler *port, uint8_t id, uint16_t address);
  int read1ByteRx     (PortHandler *port, uint8_t id, uint8_t *data, uint8_t *error = 0);
  int read1ByteTxRx   (PortHandler *port, uint8_t id, uint16_t address, uint8_t *data, uint8_t *error = 0);

  int read2ByteTx     (PortHandler *port, uint8_t id, uint16_t address);
  int read2ByteRx     (PortHandler *port, uint8_t id, uint16_t *data, uint8_t *error = 0);
  int read2ByteTxRx   (PortHandler *port, uint8_t id, uint16_t address, uint16_t *data, uint8_t *error = 0);

  int read4ByteTx     (PortHandler *port, uint8_t id, uint16_t address);
  int read4ByteRx     (PortHandler *port, uint8_t id, uint32_t *data, uint8_t *error = 0);
  int read4ByteTxRx   (PortHandler *port, uint8_t id, uint16_t address, uint32_t *data, uint8_t *error = 0);

  int writeTxOnly     (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data);
  int writeTxRx       (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);

  int write1ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint8_t data);
  int write1ByteTxRx  (PortHandler *port, uint8_t id, uint16_t address, uint8_t data, uint8_t *error = 0);

  int write2ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t data);
  int write2ByteTxRx  (PortHandler *port, uint8_t id, uint16_t address, uint16_t data, uint8_t *error = 0);

  int write4ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint32_t data);
  int write4ByteTxRx  (PortHandler *port, uint8_t id, uint16_t address, uint32_t data, uint8_t *error = 0);

  int regWriteTxOnly  (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data);
  int regWriteTxRx    (PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error = 0);

  int syncReadTx      (PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);
  // SyncReadRx   -> GroupSyncRead class
  // SyncReadTxRx -> GroupSyncRead class

  // param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
  int syncWriteTxOnly (PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);

  // param : LEN1 ID1 ADDR1 LEN2 ID2 ADDR2 ...
  int bulkReadTx      (PortHandler *port, uint8_t *param, uint16_t param_length);
  // BulkReadRx   -> GroupBulkRead class
  // BulkReadTxRx -> GroupBulkRead class

  int bulkWriteTxOnly (PortHandler *port, uint8_t *param, uint16_t param_length);
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PROTOCOL1PACKETHANDLER_H_ */
