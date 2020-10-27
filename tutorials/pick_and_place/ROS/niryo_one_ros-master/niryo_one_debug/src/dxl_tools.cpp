/*
    dxl_tools.cpp
    Copyright (C) 2018 Niryo
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
*/

#include "niryo_one_debug/dxl_tools.h"

DxlTools::DxlTools(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler)
{
    this->portHandler = portHandler;
    this->packetHandler = packetHandler;
}

int DxlTools::setupDxlBus(int baudrate)
{
    if (!portHandler->setupGpio()) {
        printf("ERROR: Failed to setup direction GPIO pin for Dynamixel half-duplex serial\n");
        return -1;
    }
    if (!portHandler->openPort()) {
        printf("Error: Failed to open Uart port for Dynamixel bus\n");
        return -1;
    }
    if (!portHandler->setBaudRate(baudrate)) {
        printf("Error: Failed to set baudrate for Dynamixel bus\n");
        return -1;
    }

    printf("Dxl Bus successfully setup\n");
    return 1;
}

void DxlTools::broadcastPing()
{
    int dxl_comm_result = COMM_TX_FAIL;
    std::vector<uint8_t> id_list;

    dxl_comm_result = packetHandler->broadcastPing(portHandler, id_list);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("Failed to scan Dynamixel bus: %d\n", dxl_comm_result);
        return;
    }

    printf("Detected Dxl motor IDs:\n");
    for (int i = 0; i < id_list.size(); i++) {
        printf("- %d\n", id_list.at(i));
    }
}

void DxlTools::ping(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    uint16_t dxl_model_number;

    dxl_comm_result = packetHandler->ping(portHandler, (uint8_t)id, &dxl_model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("Ping failed: %d\n", dxl_comm_result);
    }
    else if (dxl_error != 0) {
        printf("Ping OK for ID: %d, but an error flag is set on the motor: %d\n", id, (int)dxl_error);
    }
    else {
        printf("Ping succeeded for ID: %d\n", id);
    }
}
        
void DxlTools::setRegister(int id, int reg_address, int value, int size)
{
    int dxl_comm_result = COMM_TX_FAIL;

    if (size == 1) {
        dxl_comm_result = packetHandler->write1ByteTxOnly(portHandler, (uint8_t) id,
                (uint32_t)reg_address, (uint8_t)value);
    }
    else if (size == 2) {
        dxl_comm_result = packetHandler->write2ByteTxOnly(portHandler, (uint8_t) id,
                (uint32_t)reg_address, (uint16_t)value);
    }
    else if (size == 4) {
        dxl_comm_result = packetHandler->write4ByteTxOnly(portHandler, (uint8_t) id,
                (uint32_t)reg_address, (uint32_t)value);
    }
    else {
        printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
        return;
    }

    if (dxl_comm_result != COMM_SUCCESS) {
        printf("Failed to set register: %d\n", dxl_comm_result);
    }
    else {
        printf("Successfully sent register command\n");
    }
}

void DxlTools::closePort()
{
    portHandler->closePort();
}
