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

#include <stdio.h>
#include <algorithm>
#include "dynamixel_sdk/group_bulk_read.h"

using namespace dynamixel;

GroupBulkRead::GroupBulkRead(PortHandler *port, PacketHandler *ph)
  : port_(port),
    ph_(ph),
    last_result_(false),
    is_param_changed_(false),
    param_(0)
{
  clearParam();
}

void GroupBulkRead::makeParam()
{
  if (id_list_.size() == 0)
    return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  if (ph_->getProtocolVersion() == 1.0)
  {
    param_ = new uint8_t[id_list_.size() * 3];  // ID(1) + ADDR(1) + LENGTH(1)
  }
  else    // 2.0
  {
    param_ = new uint8_t[id_list_.size() * 5];  // ID(1) + ADDR(2) + LENGTH(2)
  }

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    if (ph_->getProtocolVersion() == 1.0)
    {
      param_[idx++] = (uint8_t)length_list_[id];    // LEN
      param_[idx++] = id;                           // ID
      param_[idx++] = (uint8_t)address_list_[id];   // ADDR
    }
    else    // 2.0
    {
      param_[idx++] = id;                               // ID
      param_[idx++] = DXL_LOBYTE(address_list_[id]);    // ADDR_L
      param_[idx++] = DXL_HIBYTE(address_list_[id]);    // ADDR_H
      param_[idx++] = DXL_LOBYTE(length_list_[id]);     // LEN_L
      param_[idx++] = DXL_HIBYTE(length_list_[id]);     // LEN_H
    }
  }
}

bool GroupBulkRead::addParam(uint8_t id, uint16_t start_address, uint16_t data_length)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  length_list_[id]    = data_length;
  address_list_[id]   = start_address;
  data_list_[id]      = new uint8_t[data_length];

  is_param_changed_   = true;
  return true;
}

void GroupBulkRead::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  address_list_.erase(id);
  length_list_.erase(id);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}

void GroupBulkRead::clearParam()
{
  if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  address_list_.clear();
  length_list_.clear();
  data_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

int GroupBulkRead::txPacket()
{
  if (id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true || param_ == 0)
    makeParam();

  if (ph_->getProtocolVersion() == 1.0)
  {
    return ph_->bulkReadTx(port_, param_, id_list_.size() * 3);
  }
  else    // 2.0
  {
    return ph_->bulkReadTx(port_, param_, id_list_.size() * 5);
  }
}

int GroupBulkRead::rxPacket()
{
  int cnt            = id_list_.size();
  int result          = COMM_RX_FAIL;

  last_result_ = false;

  if (cnt == 0)
    return COMM_NOT_AVAILABLE;

  for (int i = 0; i < cnt; i++)
  {
    uint8_t id = id_list_[i];

    result = ph_->readRx(port_, id, length_list_[id], data_list_[id]);
    if (result != COMM_SUCCESS)
      return result;
  }

  if (result == COMM_SUCCESS)
    last_result_ = true;

  return result;
}

int GroupBulkRead::txRxPacket()
{
  int result         = COMM_TX_FAIL;

  result = txPacket();
  if (result != COMM_SUCCESS)
    return result;

  return rxPacket();
}

bool GroupBulkRead::isAvailable(uint8_t id, uint16_t address, uint16_t data_length)
{
  uint16_t start_addr;

  if (last_result_ == false || data_list_.find(id) == data_list_.end())
    return false;

  start_addr = address_list_[id];

  if (address < start_addr || start_addr + length_list_[id] - data_length < address)
    return false;

  return true;
}

uint32_t GroupBulkRead::getData(uint8_t id, uint16_t address, uint16_t data_length)
{
  if (isAvailable(id, address, data_length) == false)
    return 0;

  uint16_t start_addr = address_list_[id];

  switch(data_length)
  {
    case 1:
      return data_list_[id][address - start_addr];

    case 2:
      return DXL_MAKEWORD(data_list_[id][address - start_addr], data_list_[id][address - start_addr + 1]);

    case 4:
      return DXL_MAKEDWORD(DXL_MAKEWORD(data_list_[id][address - start_addr + 0], data_list_[id][address - start_addr + 1]),
                           DXL_MAKEWORD(data_list_[id][address - start_addr + 2], data_list_[id][address - start_addr + 3]));

    default:
      return 0;
  }
}
