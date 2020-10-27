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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_H_

#ifdef __linux__
#define WINDECLSPEC
#elif defined(_WIN32) || defined(_WIN64)
#ifdef WINDLLEXPORT
#define WINDECLSPEC __declspec(dllexport)
#else
#define WINDECLSPEC __declspec(dllimport)
#endif
#endif

#include <stdint.h>

namespace dynamixel
{

class WINDECLSPEC PortHandler
{
 public:
  static const int DEFAULT_BAUDRATE_ = 1000000;

  static PortHandler *getPortHandler(const char *port_name);

  bool   is_using_;

  virtual ~PortHandler() { }

  virtual bool setupGpio() = 0;
  virtual void gpioHigh() = 0;
  virtual void gpioLow() = 0;

  virtual bool    openPort() = 0;
  virtual void    closePort() = 0;
  virtual void    clearPort() = 0;

  virtual void    setPortName(const char* port_name) = 0;
  virtual char   *getPortName() = 0;

  virtual bool    setBaudRate(const int baudrate) = 0;
  virtual int     getBaudRate() = 0;

  virtual int     getBytesAvailable() = 0;

  virtual int     readPort(uint8_t *packet, int length) = 0;
  virtual int     writePort(uint8_t *packet, int length) = 0;

  virtual void    setPacketTimeout(uint16_t packet_length) = 0;
  virtual void    setPacketTimeout(double msec) = 0;
  virtual bool    isPacketTimeout() = 0;
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_PORTHANDLER_H_ */
