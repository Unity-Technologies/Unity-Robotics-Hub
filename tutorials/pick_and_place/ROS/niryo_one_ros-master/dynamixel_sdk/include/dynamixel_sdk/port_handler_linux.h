/*
 * port_handler.h
 * Copyright (c) 2017, Niryo
 * All rights reserved.
 *
 * This library is an adaptation of dynamixel_sdk library for Raspberry Pi 3 with wiringPi
 * See license below
 */

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

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_H_


#include "dynamixel_sdk/port_handler.h"

namespace dynamixel
{

class PortHandlerLinux : public PortHandler
{
 private:
  int     socket_fd_;
  int     baudrate_;
  char    port_name_[30];

  double  packet_start_time_;
  double  packet_timeout_;
  double  tx_time_per_byte;

  bool    setupPort(const int cflag_baud);
  bool    setCustomBaudrate(int speed);
  int     getCFlagBaud(const int baudrate);

  double  getCurrentTime();
  double  getTimeSinceStart();

 public:
  PortHandlerLinux(const char *port_name);
  virtual ~PortHandlerLinux() { closePort(); }

  bool    setupGpio();
  void    gpioHigh();
  void    gpioLow();

  bool    openPort();
  void    closePort();
  void    clearPort();

  void    setPortName(const char *port_name);
  char   *getPortName();

  bool    setBaudRate(const int baudrate);
  int     getBaudRate();

  int     getBytesAvailable();

  int     readPort(uint8_t *packet, int length);
  int     writePort(uint8_t *packet, int length);

  void    setPacketTimeout(uint16_t packet_length);
  void    setPacketTimeout(double msec);
  bool    isPacketTimeout();
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERLINUX_H_ */
