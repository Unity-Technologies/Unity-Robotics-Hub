/*
    mcp_can_rpi.h
  	This library is an adaptation of MCP_CAN library for Raspberry Pi 3 with wiringPi
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	--> Below the original license (from MCP_CAN library):

	mcp_can.h
	2012 Copyright (c) Seeed Technology Inc.  All right reserved.
	2017 Copyright (c) Cory J. Fowler  All Rights Reserved.
	Author:Loovee
	Contributor: Cory J. Fowler
	2017-09-25
	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.
	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.
	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
	1301  USA
*/


#ifndef MCP_CAN_RPI_H
#define MCP_CAN_RPI_H

#ifdef __arm__
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif

// for debug
#include <stdio.h>

#include <time.h>

#include "mcp_can_rpi/mcp_can_dfs_rpi.h"
#define MAX_CHAR_IN_MESSAGE 8

#define CAN_MODEL_NUMBER 10000

class MCP_CAN
{
    private:
    
    INT8U   m_nExtFlg;                                                  // Identifier Type
                                                                        // Extended (29 bit) or Standard (11 bit)
    INT32U  m_nID;                                                      // CAN ID
    INT8U   m_nDlc;                                                     // Data Length Code
    INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];                            	// Data array
    INT8U   m_nRtr;                                                     // Remote request flag
    INT8U   m_nfilhit;                                                  // The number of the filter that matched the message
    //INT8U   MCPCS;  (NOT NEEDED, wiringPi already handles CS pin)     // Chip Select pin number 
    INT8U   mcpMode;                                                    // Mode to return to after configurations are performed.
    
    int spi_channel;
    int spi_baudrate;
    INT8U gpio_can_interrupt;

/*********************************************************************************************************
 *  mcp2515 driver function 
 *********************************************************************************************************/
   // private:
   private:

    struct timespec delay_spi_can = {0};
    
    void spiTransfer(uint8_t byte_number, unsigned char *buf);
    
    void mcp2515_reset(void);                                           // Soft Reset MCP2515

    INT8U mcp2515_readRegister(const INT8U address);                    // Read MCP2515 register
    
    void mcp2515_readRegisterS(const INT8U address,                     // Read MCP2515 successive registers
	                             INT8U values[], 
                               const INT8U n);
   
    void mcp2515_setRegister(const INT8U address,                       // Set MCP2515 register
                             const INT8U value);

    void mcp2515_setRegisterS(const INT8U address,                      // Set MCP2515 successive registers
                              const INT8U values[],
                              const INT8U n);
    
    void mcp2515_initCANBuffers(void);
    
    void mcp2515_modifyRegister(const INT8U address,                    // Set specific bit(s) of a register
                                const INT8U mask,
                                const INT8U data);

    INT8U mcp2515_readStatus(void);                                     // Read MCP2515 Status
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);                 // Set mode
    INT8U mcp2515_configRate(const INT8U canSpeed,                      // Set baudrate
                             const INT8U canClock);
                             
    INT8U mcp2515_init(const INT8U canIDMode,                           // Initialize Controller
                       const INT8U canSpeed,
                       const INT8U canClock);
		       
    void mcp2515_write_mf( const INT8U mcp_addr,                        // Write CAN Mask or Filter
                           const INT8U ext,
                           const INT32U id );
			       
    void mcp2515_write_id( const INT8U mcp_addr,                        // Write CAN ID
                           const INT8U ext,
                           const INT32U id );

    void mcp2515_read_id( const INT8U mcp_addr,                         // Read CAN ID
				INT8U* ext,
                                INT32U* id );

    void mcp2515_write_canMsg( const INT8U buffer_sidh_addr );          // Write CAN message
    void mcp2515_read_canMsg( const INT8U buffer_sidh_addr);            // Read CAN message
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);                     // Find empty transmit buffer

/*********************************************************************************************************
 *  CAN operator function
 *********************************************************************************************************/

    INT8U setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData);        // Set message
    INT8U clearMsg();                                                   // Clear all message to zero
    INT8U readMsg();                                                    // Read message
    INT8U sendMsg();                                                    // Send message

public:
    MCP_CAN(int spi_channel, int spi_baudrate, INT8U gpio_can_interrupt);
    INT8U begin(INT8U idmodeset, INT8U speedset, INT8U clockset);       // Initilize controller prameters
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);               // Initilize Mask(s)
    INT8U init_Mask(INT8U num, INT32U ulData);                          // Initilize Mask(s)
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);               // Initilize Filter(s)
    INT8U init_Filt(INT8U num, INT32U ulData);                          // Initilize Filter(s)
    INT8U setMode(INT8U opMode);                                        // Set operational mode
    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);      // Send message to transmit buffer
    INT8U sendMsgBuf(INT32U id, INT8U len, INT8U *buf);                 // Send message to transmit buffer
    INT8U readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U *buf);   // Read message from receive buffer
    INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U *buf);               // Read message from receive buffer
    INT8U checkReceive(void);                                           // Check for received data
    INT8U checkError(void);                                             // Check for errors
    INT8U getError(void);                                               // Check for errors
    INT8U errorCountRX(void);                                           // Get error count
    INT8U errorCountTX(void);                                           // Get error count
    INT8U enOneShotTX(void);                                            // Enable one-shot transmission
    INT8U disOneShotTX(void);                                           // Disable one-shot transmission

    bool setupInterruptGpio();
    bool setupSpi();
    bool canReadData();
};

#endif
