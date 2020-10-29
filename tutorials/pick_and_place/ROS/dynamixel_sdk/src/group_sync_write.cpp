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
#include "dynamixel_sdk/group_sync_write.h"

using namespace dynamixel;

GroupSyncWrite::GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : port_(port),
    ph_(ph),
    is_param_changed_(false),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

void GroupSyncWrite::makeParam()
{
  if (id_list_.size() == 0) return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  param_ = new uint8_t[id_list_.size() * (1 + data_length_)]; // ID(1) + DATA(data_length)

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    if (data_list_[id] == 0)
      return;

    param_[idx++] = id;
    for (int c = 0; c < data_length_; c++)
      param_[idx++] = (data_list_[id])[c];
  }
}

bool GroupSyncWrite::addParam(uint8_t id, uint8_t *data)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void GroupSyncWrite::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}

bool GroupSyncWrite::changeParam(uint8_t id, uint8_t *data)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return false;

  delete[] data_list_[id];
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void GroupSyncWrite::clearParam()
{
  if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  data_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

int GroupSyncWrite::txPacket()
{
  if (id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;

  if (is_param_changed_ == true || param_ == 0)
    makeParam();

  return ph_->syncWriteTxOnly(port_, start_address_, data_length_, param_, id_list_.size() * (1 + data_length_));
}
