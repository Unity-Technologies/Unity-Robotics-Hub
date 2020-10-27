/*
    dxl_tools.h
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

#ifndef DXL_TOOLS_H
#define DXL_TOOLS_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <string>
#include <vector>

class DxlTools {

    protected:
        dynamixel::PortHandler* portHandler;
        dynamixel::PacketHandler* packetHandler;

    public:
        DxlTools() {}
        DxlTools(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler);

        int setupDxlBus(int baudrate);
        void broadcastPing();
        void ping(int id);
        void setRegister(int id, int reg_address, int value, int size);

        void closePort();

};

#endif
