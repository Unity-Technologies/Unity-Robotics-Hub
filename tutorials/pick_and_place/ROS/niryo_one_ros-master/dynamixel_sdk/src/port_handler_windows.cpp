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

/* Author: Ryu Woon Jung (Leon) */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include "dynamixel_sdk/port_handler_windows.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#define LATENCY_TIMER  16 // msec (USB latency timer)

using namespace dynamixel;

PortHandlerWindows::PortHandlerWindows(const char *port_name)
  : serial_handle_(INVALID_HANDLE_VALUE),
  baudrate_(DEFAULT_BAUDRATE_),
  packet_start_time_(0.0),
  packet_timeout_(0.0),
  tx_time_per_byte_(0.0)
{
  is_using_ = false;

  char buffer[15];
  sprintf_s(buffer, sizeof(buffer), "\\\\.\\%s", port_name);
  setPortName(buffer);
}

//#include <fcntl.h>
//#include <wiringPi.h>
//#include <time.h>

bool PortHandlerWindows::setupGpio() { return false; }

void PortHandlerWindows::gpioHigh() { }

void PortHandlerWindows::gpioLow() { }

bool PortHandlerWindows::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandlerWindows::closePort()
{
  if (serial_handle_ != INVALID_HANDLE_VALUE)
  {
    CloseHandle(serial_handle_);
    serial_handle_ = INVALID_HANDLE_VALUE;
  }
}

void PortHandlerWindows::clearPort()
{
  PurgeComm(serial_handle_, PURGE_RXABORT | PURGE_RXCLEAR);
}

void PortHandlerWindows::setPortName(const char *port_name)
{
  strcpy_s(port_name_, sizeof(port_name_), port_name);
}

char *PortHandlerWindows::getPortName()
{
  return port_name_;
}

bool PortHandlerWindows::setBaudRate(const int baudrate)
{
  closePort();

  baudrate_ = baudrate;
  return setupPort(baudrate);
}

int PortHandlerWindows::getBaudRate()
{
  return baudrate_;
}

int PortHandlerWindows::getBytesAvailable()
{
  DWORD retbyte = 2;
  BOOL res = DeviceIoControl(serial_handle_, GENERIC_READ | GENERIC_WRITE, NULL, 0, 0, 0, &retbyte, (LPOVERLAPPED)NULL);

  printf("%d", (int)res);
  return (int)retbyte;
}

int PortHandlerWindows::readPort(uint8_t *packet, int length)
{
  DWORD dwRead = 0;

  if (ReadFile(serial_handle_, packet, (DWORD)length, &dwRead, NULL) == FALSE)
    return -1;

  return (int)dwRead;
}

int PortHandlerWindows::writePort(uint8_t *packet, int length)
{
  DWORD dwWrite = 0;

  if (WriteFile(serial_handle_, packet, (DWORD)length, &dwWrite, NULL) == FALSE)
    return -1;

  return (int)dwWrite;
}

void PortHandlerWindows::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_ = getCurrentTime();
  packet_timeout_ = (tx_time_per_byte_ * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerWindows::setPacketTimeout(double msec)
{
  packet_start_time_ = getCurrentTime();
  packet_timeout_ = msec;
}

bool PortHandlerWindows::isPacketTimeout()
{
  if (getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandlerWindows::getCurrentTime()
{
  QueryPerformanceCounter(&counter_);
  QueryPerformanceFrequency(&freq_);
  return (double)counter_.QuadPart / (double)freq_.QuadPart * 1000.0;
}

double PortHandlerWindows::getTimeSinceStart()
{
  double time;

  time = getCurrentTime() - packet_start_time_;
  if (time < 0.0) packet_start_time_ = getCurrentTime();

  return time;
}

bool PortHandlerWindows::setupPort(int baudrate)
{
  DCB dcb;
  COMMTIMEOUTS timeouts;
  DWORD dwError;

  closePort();

  serial_handle_ = CreateFileA(port_name_, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (serial_handle_ == INVALID_HANDLE_VALUE)
  {
    printf("[PortHandlerWindows::SetupPort] Error opening serial port!\n");
    return false;
  }

  dcb.DCBlength = sizeof(DCB);
  if (GetCommState(serial_handle_, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  // Set baudrate
  dcb.BaudRate = (DWORD)baudrate;
  dcb.ByteSize = 8;                    // Data bit = 8bit
  dcb.Parity = NOPARITY;             // No parity
  dcb.StopBits = ONESTOPBIT;           // Stop bit = 1
  dcb.fParity = NOPARITY;             // No Parity check
  dcb.fBinary = 1;                    // Binary mode
  dcb.fNull = 0;                    // Get Null byte
  dcb.fAbortOnError = 0;
  dcb.fErrorChar = 0;
  // Not using XOn/XOff
  dcb.fOutX = 0;
  dcb.fInX = 0;
  // Not using H/W flow control
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fDsrSensitivity = 0;
  dcb.fOutxDsrFlow = 0;
  dcb.fOutxCtsFlow = 0;

  if (SetCommState(serial_handle_, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (SetCommMask(serial_handle_, 0) == FALSE) // Not using Comm event
    goto DXL_HAL_OPEN_ERROR;
  if (SetupComm(serial_handle_, 4096, 4096) == FALSE) // Buffer size (Rx,Tx)
    goto DXL_HAL_OPEN_ERROR;
  if (PurgeComm(serial_handle_, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR) == FALSE) // Clear buffer
    goto DXL_HAL_OPEN_ERROR;
  if (ClearCommError(serial_handle_, &dwError, NULL) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (GetCommTimeouts(serial_handle_, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;
  // Timeout (Not using timeout)
  // Immediatly return
  timeouts.ReadIntervalTimeout = 0;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 1; // must not be zero.
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  if (SetCommTimeouts(serial_handle_, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  tx_time_per_byte_ = (1000.0 / (double)baudrate_) * 10.0;
  return true;

DXL_HAL_OPEN_ERROR:
  closePort();
  return false;
}
