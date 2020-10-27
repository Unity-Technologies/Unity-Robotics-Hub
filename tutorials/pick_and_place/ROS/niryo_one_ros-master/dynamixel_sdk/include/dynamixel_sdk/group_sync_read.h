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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_


#include <map>
#include <vector>
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/packet_handler.h"

namespace dynamixel
{

class WINDECLSPEC GroupSyncRead
{
 private:
  PortHandler    *port_;
  PacketHandler  *ph_;

  std::vector<uint8_t>            id_list_;
  std::map<uint8_t, uint8_t* >    data_list_; // <id, data>

  bool            last_result_;
  bool            is_param_changed_;

  uint8_t        *param_;
  uint16_t        start_address_;
  uint16_t        data_length_;

  void    makeParam();

 public:
  GroupSyncRead(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);
  ~GroupSyncRead() { clearParam(); }

  PortHandler     *getPortHandler()   { return port_; }
  PacketHandler   *getPacketHandler() { return ph_; }

  bool    addParam    (uint8_t id);
  void    removeParam (uint8_t id);
  void    clearParam  ();

  int     txPacket();
  int     rxPacket();
  int     txRxPacket();

  bool        isAvailable (uint8_t id, uint16_t address, uint16_t data_length);
  uint32_t    getData     (uint8_t id, uint16_t address, uint16_t data_length);
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPSYNCREAD_H_ */
