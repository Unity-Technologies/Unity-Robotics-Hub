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

#include <algorithm>
#include "dynamixel_sdk/group_sync_read.h"

using namespace dynamixel;

GroupSyncRead::GroupSyncRead(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : port_(port),
    ph_(ph),
    last_result_(false),
    is_param_changed_(false),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

void GroupSyncRead::makeParam()
{
  if (ph_->getProtocolVersion() == 1.0 || id_list_.size() == 0)
    return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  param_ = new uint8_t[id_list_.size() * 1];  // ID(1)

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
    param_[idx++] = id_list_[i];
}

bool GroupSyncRead::addParam(uint8_t id)
{
  if (ph_->getProtocolVersion() == 1.0)
    return false;

  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id] = new uint8_t[data_length_];

  is_param_changed_   = true;
  return true;
}
void GroupSyncRead::removeParam(uint8_t id)
{
  if (ph_->getProtocolVersion() == 1.0)
    return;

  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}
void GroupSyncRead::clearParam()
{
  if (ph_->getProtocolVersion() == 1.0 || id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  data_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

int GroupSyncRead::txPacket()
{
  if (ph_->getProtocolVersion() == 1.0 || id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true || param_ == 0)
    makeParam();

  return ph_->syncReadTx(port_, start_address_, data_length_, param_, (uint16_t)id_list_.size() * 1);
}

int GroupSyncRead::rxPacket()
{
  last_result_ = false;

  if (ph_->getProtocolVersion() == 1.0)
    return COMM_NOT_AVAILABLE;

  int cnt            = id_list_.size();
  int result         = COMM_RX_FAIL;

  if (cnt == 0)
    return COMM_NOT_AVAILABLE;

  for (int i = 0; i < cnt; i++)
  {
    uint8_t id = id_list_[i];

    result = ph_->readRx(port_, id, data_length_, data_list_[id]);
    if (result != COMM_SUCCESS)
      return result;
  }

  if (result == COMM_SUCCESS)
    last_result_ = true;

  return result;
}

int GroupSyncRead::txRxPacket()
{
  if (ph_->getProtocolVersion() == 1.0)
    return COMM_NOT_AVAILABLE;

  int result         = COMM_TX_FAIL;

  result = txPacket();
  if (result != COMM_SUCCESS)
    return result;

  return rxPacket();
}

bool GroupSyncRead::isAvailable(uint8_t id, uint16_t address, uint16_t data_length)
{
  if (ph_->getProtocolVersion() == 1.0 || last_result_ == false || data_list_.find(id) == data_list_.end())
    return false;

  if (address < start_address_ || start_address_ + data_length_ - data_length < address)
    return false;

  return true;
}

uint32_t GroupSyncRead::getData(uint8_t id, uint16_t address, uint16_t data_length)
{
  if (isAvailable(id, address, data_length) == false)
    return 0;

  switch(data_length)
  {
    case 1:
      return data_list_[id][address - start_address_];

    case 2:
      return DXL_MAKEWORD(data_list_[id][address - start_address_], data_list_[id][address - start_address_ + 1]);

    case 4:
      return DXL_MAKEDWORD(DXL_MAKEWORD(data_list_[id][address - start_address_ + 0], data_list_[id][address - start_address_ + 1]),
                 DXL_MAKEWORD(data_list_[id][address - start_address_ + 2], data_list_[id][address - start_address_ + 3]));

    default:
      return 0;
  }
}
