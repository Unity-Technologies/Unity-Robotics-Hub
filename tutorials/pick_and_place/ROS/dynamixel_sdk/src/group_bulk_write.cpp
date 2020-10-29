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
#include "dynamixel_sdk/group_bulk_write.h"

using namespace dynamixel;

GroupBulkWrite::GroupBulkWrite(PortHandler *port, PacketHandler *ph)
  : port_(port),
    ph_(ph),
    is_param_changed_(false),
    param_(0),
    param_length_(0)
{
  clearParam();
}

void GroupBulkWrite::makeParam()
{
  if (ph_->getProtocolVersion() == 1.0 || id_list_.size() == 0)
    return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  param_length_ = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
    param_length_ += 1 + 2 + 2 + length_list_[id_list_[i]];

  param_ = new uint8_t[param_length_];

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    if (data_list_[id] == 0)
      return;

    param_[idx++] = id;
    param_[idx++] = DXL_LOBYTE(address_list_[id]);
    param_[idx++] = DXL_HIBYTE(address_list_[id]);
    param_[idx++] = DXL_LOBYTE(length_list_[id]);
    param_[idx++] = DXL_HIBYTE(length_list_[id]);
    for (int c = 0; c < length_list_[id]; c++)
      param_[idx++] = (data_list_[id])[c];
  }
}

bool GroupBulkWrite::addParam(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data)
{
  if (ph_->getProtocolVersion() == 1.0)
    return false;

  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  address_list_[id]   = start_address;
  length_list_[id]    = data_length;
  data_list_[id]      = new uint8_t[data_length];
  for (int c = 0; c < data_length; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}
void GroupBulkWrite::removeParam(uint8_t id)
{
  if (ph_->getProtocolVersion() == 1.0)
    return;

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
bool GroupBulkWrite::changeParam(uint8_t id, uint16_t start_address, uint16_t data_length, uint8_t *data)
{
  if (ph_->getProtocolVersion() == 1.0)
    return false;

  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return false;

  address_list_[id]   = start_address;
  length_list_[id]    = data_length;
  delete[] data_list_[id];
  data_list_[id]      = new uint8_t[data_length];
  for (int c = 0; c < data_length; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}
void GroupBulkWrite::clearParam()
{
  if (ph_->getProtocolVersion() == 1.0 || id_list_.size() == 0)
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
int GroupBulkWrite::txPacket()
{
  if (ph_->getProtocolVersion() == 1.0 || id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true || param_ == 0)
    makeParam();

  return ph_->bulkWriteTxOnly(port_, param_, param_length_);
}
