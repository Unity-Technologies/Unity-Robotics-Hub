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

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include <string.h>
#include <stdlib.h>
#include "dynamixel_sdk/protocol1_packet_handler.h"

#define TXPACKET_MAX_LEN    (250)
#define RXPACKET_MAX_LEN    (250)

///////////////// for Protocol 1.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      64      // Undefined instruction or delivering the action command without the reg_write command.

using namespace dynamixel;

Protocol1PacketHandler *Protocol1PacketHandler::unique_instance_ = new Protocol1PacketHandler();

Protocol1PacketHandler::Protocol1PacketHandler() { }

void Protocol1PacketHandler::printTxRxResult(int result)
{
  switch(result)
  {
  case COMM_SUCCESS:
    printf("[TxRxResult] Communication success.\n");
    break;

  case COMM_PORT_BUSY:
    printf("[TxRxResult] Port is in use!\n");
    break;

  case COMM_TX_FAIL:
    printf("[TxRxResult] Failed transmit instruction packet!\n");
    break;

  case COMM_RX_FAIL:
    printf("[TxRxResult] Failed get status packet from device!\n");
    break;

  case COMM_TX_ERROR:
    printf("[TxRxResult] Incorrect instruction packet!\n");
    break;

  case COMM_RX_WAITING:
    printf("[TxRxResult] Now recieving status packet!\n");
    break;

  case COMM_RX_TIMEOUT:
    printf("[TxRxResult] There is no status packet!\n");
    break;

  case COMM_RX_CORRUPT:
    printf("[TxRxResult] Incorrect status packet!\n");
    break;

  case COMM_NOT_AVAILABLE:
    printf("[TxRxResult] Protocol does not support This function!\n");
    break;

  default:
    break;
  }
}

void Protocol1PacketHandler::printRxPacketError(uint8_t error)
{
  if (error & ERRBIT_VOLTAGE)
    printf("[RxPacketError] Input voltage error!\n");

  if (error & ERRBIT_ANGLE)
    printf("[RxPacketError] Angle limit error!\n");

  if (error & ERRBIT_OVERHEAT)
    printf("[RxPacketError] Overheat error!\n");

  if (error & ERRBIT_RANGE)
    printf("[RxPacketError] Out of range error!\n");

  if (error & ERRBIT_CHECKSUM)
    printf("[RxPacketError] Checksum error!\n");

  if (error & ERRBIT_OVERLOAD)
    printf("[RxPacketError] Overload error!\n");

  if (error & ERRBIT_INSTRUCTION)
    printf("[RxPacketError] Instruction code error!\n");
}

int Protocol1PacketHandler::txPacket(PortHandler *port, uint8_t *txpacket)
{
  uint8_t checksum               = 0;
  uint8_t total_packet_length    = txpacket[PKT_LENGTH] + 4; // 4: HEADER0 HEADER1 ID LENGTH
  uint8_t written_packet_length  = 0;

  if (port->is_using_)
    return COMM_PORT_BUSY;
  port->is_using_ = true;

  // check max packet length
  if (total_packet_length > TXPACKET_MAX_LEN)
  {
    port->is_using_ = false;
    return COMM_TX_ERROR;
  }

  // make packet header
  txpacket[PKT_HEADER0]   = 0xFF;
  txpacket[PKT_HEADER1]   = 0xFF;

  // add a checksum to the packet
  for (int idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
    checksum += txpacket[idx];
  txpacket[total_packet_length - 1] = ~checksum;

  // tx packet
  port->clearPort();
  written_packet_length = port->writePort(txpacket, total_packet_length);
  if (total_packet_length != written_packet_length)
  {
    port->is_using_ = false;
    return COMM_TX_FAIL;
  }

  return COMM_SUCCESS;
}

int Protocol1PacketHandler::rxPacket(PortHandler *port, uint8_t *rxpacket)
{
  int     result         = COMM_TX_FAIL;

  uint8_t checksum       = 0;
  uint8_t rx_length      = 0;
  uint8_t wait_length    = 6;    // minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

  while(true)
  {
    rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
    if (rx_length >= wait_length)
    {
      uint8_t idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 1); idx++)
      {
        if (rxpacket[idx] == 0xFF && rxpacket[idx+1] == 0xFF)
          break;
      }

      if (idx == 0)   // found at the beginning of the packet
      {
        if (rxpacket[PKT_ID] > 0xFD ||                   // unavailable ID
           rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN ||   // unavailable Length
           rxpacket[PKT_ERROR] >= 0x64)                 // unavailable Error
        {
            // remove the first byte in the packet
            for (uint8_t s = 0; s < rx_length - 1; s++)
              rxpacket[s] = rxpacket[1 + s];
            //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
            rx_length -= 1;
            continue;
        }

        // re-calculate the exact length of the rx packet
        if (wait_length != rxpacket[PKT_LENGTH] + PKT_LENGTH + 1)
        {
          wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1;
          continue;
        }

        if (rx_length < wait_length)
        {
          // check timeout
          if (port->isPacketTimeout() == true)
          {
            if (rx_length == 0)
            {
              result = COMM_RX_TIMEOUT;
            }
            else
            {
              result = COMM_RX_CORRUPT;
            }
            break;
          }
          else
          {
            continue;
          }
        }

        // calculate checksum
        for (int i = 2; i < wait_length - 1; i++)   // except header, checksum
          checksum += rxpacket[i];
        checksum = ~checksum;

        // verify checksum
        if (rxpacket[wait_length - 1] == checksum)
        {
          result = COMM_SUCCESS;
        }
        else
        {
          result = COMM_RX_CORRUPT;
        }
        break;
      }
      else
      {
        // remove unnecessary packets
        for (uint8_t s = 0; s < rx_length - idx; s++)
          rxpacket[s] = rxpacket[idx + s];
        //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
        rx_length -= idx;
      }
    }
    else
    {
      // check timeout
      if (port->isPacketTimeout() == true)
      {
        if (rx_length == 0)
        {
          result = COMM_RX_TIMEOUT;
        }
        else
        {
          result = COMM_RX_CORRUPT;
        }
        break;
      }
    }
  }
  port->is_using_ = false;

  return result;
}

// NOT for BulkRead instruction
int Protocol1PacketHandler::txRxPacket(PortHandler *port, uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error)
{
  int result = COMM_TX_FAIL;

  // tx packet
  result = txPacket(port, txpacket);
  if (result != COMM_SUCCESS)
    return result;

  // (Instruction == BulkRead) == this function is not available.
  if(txpacket[PKT_INSTRUCTION] == INST_BULK_READ)
    result = COMM_NOT_AVAILABLE;

  // (ID == Broadcast ID) == no need to wait for status packet or not available
  // (Instruction == action) == no need to wait for status packet
  if (txpacket[PKT_ID] == BROADCAST_ID || txpacket[PKT_INSTRUCTION] == INST_ACTION)
  {
    port->is_using_ = false;
    return result;
  }

  // set packet timeout
  if (txpacket[PKT_INSTRUCTION] == INST_READ)
  {
    port->setPacketTimeout((uint16_t)(txpacket[PKT_PARAMETER0+1] + 6));
  }
  else
  {
    port->setPacketTimeout((uint16_t)6); // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM
  }

  // rx packet
  do {
    result = rxPacket(port, rxpacket);
  } while (result == COMM_SUCCESS && txpacket[PKT_ID] != rxpacket[PKT_ID]);

  if (result == COMM_SUCCESS && txpacket[PKT_ID] == rxpacket[PKT_ID])
  {
    if (error != 0)
      *error = (uint8_t)rxpacket[PKT_ERROR];
  }

  return result;
}

int Protocol1PacketHandler::ping(PortHandler *port, uint8_t id, uint8_t *error)
{
  return ping(port, id, 0, error);
}

int Protocol1PacketHandler::ping(PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error)
{
  int result                 = COMM_TX_FAIL;

  uint8_t txpacket[6]         = {0};
  uint8_t rxpacket[6]         = {0};

  if (id >= BROADCAST_ID)
    return COMM_NOT_AVAILABLE;

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = 2;
  txpacket[PKT_INSTRUCTION]   = INST_PING;

  result = txRxPacket(port, txpacket, rxpacket, error);
  if (result == COMM_SUCCESS && model_number != 0)
  {
    uint8_t data_read[2] = {0};
    result = readTxRx(port, id, 0, 2, data_read);  // Address 0 : Model Number
    if (result == COMM_SUCCESS) *model_number = DXL_MAKEWORD(data_read[0], data_read[1]);
  }

  return result;
}

int Protocol1PacketHandler::broadcastPing(PortHandler *port, std::vector<uint8_t> &id_list)
{
  return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::action(PortHandler *port, uint8_t id)
{
  uint8_t txpacket[6]         = {0};

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = 2;
  txpacket[PKT_INSTRUCTION]   = INST_ACTION;

  return txRxPacket(port, txpacket, 0);
}

int Protocol1PacketHandler::reboot(PortHandler *port, uint8_t id, uint8_t *error)
{
  return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::factoryReset(PortHandler *port, uint8_t id, uint8_t option, uint8_t *error)
{
  uint8_t txpacket[6]         = {0};
  uint8_t rxpacket[6]         = {0};

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = 2;
  txpacket[PKT_INSTRUCTION]   = INST_FACTORY_RESET;

  return txRxPacket(port, txpacket, rxpacket, error);
}

int Protocol1PacketHandler::readTx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length)
{
  int result                 = COMM_TX_FAIL;

  uint8_t txpacket[8]         = {0};

  if (id >= BROADCAST_ID)
    return COMM_NOT_AVAILABLE;

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = 4;
  txpacket[PKT_INSTRUCTION]   = INST_READ;
  txpacket[PKT_PARAMETER0+0]  = (uint8_t)address;
  txpacket[PKT_PARAMETER0+1]  = (uint8_t)length;

  result = txPacket(port, txpacket);

  // set packet timeout
  if (result == COMM_SUCCESS)
    port->setPacketTimeout((uint16_t)(length+6));

  return result;
}

int Protocol1PacketHandler::readRx(PortHandler *port, uint8_t id, uint16_t length, uint8_t *data, uint8_t *error)
{
  int result                  = COMM_TX_FAIL;
  uint8_t *rxpacket           = (uint8_t *)malloc(RXPACKET_MAX_LEN);//(length+6);
  //uint8_t *rxpacket           = new uint8_t[length+6];

  do {
    result = rxPacket(port, rxpacket);
  } while (result == COMM_SUCCESS && rxpacket[PKT_ID] != id);

  if (result == COMM_SUCCESS && rxpacket[PKT_ID] == id)
  {
    if (error != 0)
    {
      *error = (uint8_t)rxpacket[PKT_ERROR];
    }
    for (uint8_t s = 0; s < length; s++)
    {
      data[s] = rxpacket[PKT_PARAMETER0 + s];
    }
    //memcpy(data, &rxpacket[PKT_PARAMETER0], length);
  }

  free(rxpacket);
  //delete[] rxpacket;
  return result;
}

int Protocol1PacketHandler::readTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  int result = COMM_TX_FAIL;

  uint8_t txpacket[8]         = {0};
  uint8_t *rxpacket           = (uint8_t *)malloc(RXPACKET_MAX_LEN);//(length+6);

  if (id >= BROADCAST_ID)
    return COMM_NOT_AVAILABLE;

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = 4;
  txpacket[PKT_INSTRUCTION]   = INST_READ;
  txpacket[PKT_PARAMETER0+0]  = (uint8_t)address;
  txpacket[PKT_PARAMETER0+1]  = (uint8_t)length;

  result = txRxPacket(port, txpacket, rxpacket, error);
  if (result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      *error = (uint8_t)rxpacket[PKT_ERROR];
    }
    for (uint8_t s = 0; s < length; s++)
    {
      data[s] = rxpacket[PKT_PARAMETER0 + s];
    }
    //memcpy(data, &rxpacket[PKT_PARAMETER0], length);
  }

  free(rxpacket);
  //delete[] rxpacket;
  return result;
}

int Protocol1PacketHandler::read1ByteTx(PortHandler *port, uint8_t id, uint16_t address)
{
  return readTx(port, id, address, 1);
}
int Protocol1PacketHandler::read1ByteRx(PortHandler *port, uint8_t id, uint8_t *data, uint8_t *error)
{
  uint8_t data_read[1] = {0};
  int result = readRx(port, id, 1, data_read, error);
  if (result == COMM_SUCCESS)
    *data = data_read[0];
  return result;
}
int Protocol1PacketHandler::read1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t *data, uint8_t *error)
{
  uint8_t data_read[1] = {0};
  int result = readTxRx(port, id, address, 1, data_read, error);
  if (result == COMM_SUCCESS)
    *data = data_read[0];
  return result;
}

int Protocol1PacketHandler::read2ByteTx(PortHandler *port, uint8_t id, uint16_t address)
{
  return readTx(port, id, address, 2);
}
int Protocol1PacketHandler::read2ByteRx(PortHandler *port, uint8_t id, uint16_t *data, uint8_t *error)
{
  uint8_t data_read[2] = {0};
  int result = readRx(port, id, 2, data_read, error);
  if (result == COMM_SUCCESS)
    *data = DXL_MAKEWORD(data_read[0], data_read[1]);
  return result;
}
int Protocol1PacketHandler::read2ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t *data, uint8_t *error)
{
  uint8_t data_read[2] = {0};
  int result = readTxRx(port, id, address, 2, data_read, error);
  if (result == COMM_SUCCESS)
    *data = DXL_MAKEWORD(data_read[0], data_read[1]);
  return result;
}

int Protocol1PacketHandler::read4ByteTx(PortHandler *port, uint8_t id, uint16_t address)
{
  return readTx(port, id, address, 4);
}
int Protocol1PacketHandler::read4ByteRx(PortHandler *port, uint8_t id, uint32_t *data, uint8_t *error)
{
  uint8_t data_read[4] = {0};
  int result = readRx(port, id, 4, data_read, error);
  if (result == COMM_SUCCESS)
    *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
  return result;
}
int Protocol1PacketHandler::read4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint32_t *data, uint8_t *error)
{
  uint8_t data_read[4] = {0};
  int result = readTxRx(port, id, address, 4, data_read, error);
  if (result == COMM_SUCCESS)
    *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
  return result;
}

int Protocol1PacketHandler::writeTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(length+7);
  //uint8_t *txpacket           = new uint8_t[length+7];

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = length+3;
  txpacket[PKT_INSTRUCTION]   = INST_WRITE;
  txpacket[PKT_PARAMETER0]    = (uint8_t)address;

  for (uint8_t s = 0; s < length; s++)
    txpacket[PKT_PARAMETER0+1+s] = data[s];
  //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

  result = txPacket(port, txpacket);
  port->is_using_ = false;

  free(txpacket);
  //delete[] txpacket;
  return result;
}

int Protocol1PacketHandler::writeTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(length+7); //#6->7
  //uint8_t *txpacket           = new uint8_t[length+7];
  uint8_t rxpacket[6]         = {0};

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = length+3;
  txpacket[PKT_INSTRUCTION]   = INST_WRITE;
  txpacket[PKT_PARAMETER0]    = (uint8_t)address;

  for (uint8_t s = 0; s < length; s++)
    txpacket[PKT_PARAMETER0+1+s] = data[s];
  //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

  result = txRxPacket(port, txpacket, rxpacket, error);

  free(txpacket);
  //delete[] txpacket;
  return result;
}

int Protocol1PacketHandler::write1ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint8_t data)
{
  uint8_t data_write[1] = { data };
  return writeTxOnly(port, id, address, 1, data_write);
}
int Protocol1PacketHandler::write1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t data, uint8_t *error)
{
  uint8_t data_write[1] = { data };
  return writeTxRx(port, id, address, 1, data_write, error);
}

int Protocol1PacketHandler::write2ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t data)
{
  uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
  return writeTxOnly(port, id, address, 2, data_write);
}
int Protocol1PacketHandler::write2ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t data, uint8_t *error)
{
  uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
  return writeTxRx(port, id, address, 2, data_write, error);
}

int Protocol1PacketHandler::write4ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint32_t data)
{
  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  return writeTxOnly(port, id, address, 4, data_write);
}
int Protocol1PacketHandler::write4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint32_t data, uint8_t *error)
{
  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  return writeTxRx(port, id, address, 4, data_write, error);
}

int Protocol1PacketHandler::regWriteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(length+6);
  //uint8_t *txpacket           = new uint8_t[length+6];

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = length+3;
  txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE;
  txpacket[PKT_PARAMETER0]    = (uint8_t)address;

  for (uint8_t s = 0; s < length; s++)
    txpacket[PKT_PARAMETER0+1+s] = data[s];
  //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

  result = txPacket(port, txpacket);
  port->is_using_ = false;

  free(txpacket);
  //delete[] txpacket;
  return result;
}

int Protocol1PacketHandler::regWriteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(length+6);
  //uint8_t *txpacket           = new uint8_t[length+6];
  uint8_t rxpacket[6]         = {0};

  txpacket[PKT_ID]            = id;
  txpacket[PKT_LENGTH]        = length+3;
  txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE;
  txpacket[PKT_PARAMETER0]    = (uint8_t)address;

  for (uint8_t s = 0; s < length; s++)
    txpacket[PKT_PARAMETER0+1+s] = data[s];
  //memcpy(&txpacket[PKT_PARAMETER0+1], data, length);

  result = txRxPacket(port, txpacket, rxpacket, error);

  free(txpacket);
  //delete[] txpacket;
  return result;
}

int Protocol1PacketHandler::syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length)
{
  return COMM_NOT_AVAILABLE;
}

int Protocol1PacketHandler::syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(param_length+8);
  // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM
  //uint8_t *txpacket           = new uint8_t[param_length + 8];

  txpacket[PKT_ID]            = BROADCAST_ID;
  txpacket[PKT_LENGTH]        = param_length + 4; // 4: INST START_ADDR DATA_LEN ... CHKSUM
  txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
  txpacket[PKT_PARAMETER0+0]  = start_address;
  txpacket[PKT_PARAMETER0+1]  = data_length;

  for (uint8_t s = 0; s < param_length; s++)
    txpacket[PKT_PARAMETER0+2+s] = param[s];
  //memcpy(&txpacket[PKT_PARAMETER0+2], param, param_length);

  result = txRxPacket(port, txpacket, 0, 0);

  free(txpacket);
  //delete[] txpacket;
  return result;
}

int Protocol1PacketHandler::bulkReadTx(PortHandler *port, uint8_t *param, uint16_t param_length)
{
  int result                 = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(param_length+7);
  // 7: HEADER0 HEADER1 ID LEN INST 0x00 ... CHKSUM
  //uint8_t *txpacket           = new uint8_t[param_length + 7];

  txpacket[PKT_ID]            = BROADCAST_ID;
  txpacket[PKT_LENGTH]        = param_length + 3; // 3: INST 0x00 ... CHKSUM
  txpacket[PKT_INSTRUCTION]   = INST_BULK_READ;
  txpacket[PKT_PARAMETER0+0]  = 0x00;

  for (uint8_t s = 0; s < param_length; s++)
    txpacket[PKT_PARAMETER0+1+s] = param[s];
  //memcpy(&txpacket[PKT_PARAMETER0+1], param, param_length);

  result = txPacket(port, txpacket);
  if (result == COMM_SUCCESS)
  {
    int wait_length = 0;
    for (int i = 0; i < param_length; i += 3)
      wait_length += param[i] + 7;
    port->setPacketTimeout((uint16_t)wait_length);
  }

  free(txpacket);
  //delete[] txpacket;
  return result;
}

int Protocol1PacketHandler::bulkWriteTxOnly(PortHandler *port, uint8_t *param, uint16_t param_length)
{
  return COMM_NOT_AVAILABLE;
}
