/*
    dxl_driver.cpp
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

#include "niryo_one_driver/xl320_driver.h"

DxlDriver::DxlDriver(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler)
{
    this->portHandler = portHandler;
    this->packetHandler = packetHandler;
}

int DxlDriver::ping(uint8_t id)
{
    uint8_t dxl_error = 0;
    
    int result = packetHandler->ping(portHandler, id, &dxl_error);
    
    if (dxl_error != 0) {
        return dxl_error; 
    }

    return result;
}

int DxlDriver::getModelNumber(uint8_t id, uint16_t *dxl_model_number)
{
    uint8_t dxl_error = 0;

    int result = packetHandler->ping(portHandler, id, dxl_model_number, &dxl_error);
    
    if (dxl_error != 0) {
        return dxl_error; 
    }

    return result;
}

int DxlDriver::scan(std::vector<uint8_t> &id_list) 
{
    return packetHandler->broadcastPing(portHandler, id_list);
}

int DxlDriver::reboot(uint8_t id)
{
    uint8_t dxl_error = 0;
    
    int result = packetHandler->reboot(portHandler, id, &dxl_error);

    if (dxl_error != 0) {
        return dxl_error;
    }

    return result;
}

/*
 *  -----------------   SYNC WRITE   --------------------
 */

int DxlDriver::syncWrite1Byte(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, DXL_LEN_ONE_BYTE);

    if (id_list.size() != data_list.size()) {
        return LEN_ID_DATA_NOT_SAME; 
    }

    if (id_list.size() == 0) {
        return COMM_SUCCESS;
    }
    
    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id=id_list.begin(), it_data=data_list.begin() ; 
            it_id < id_list.end() && it_data < data_list.end() ; 
            it_id++, it_data++) {
        uint8_t params[1] = { (uint8_t)(*it_data) };
        if (!groupSyncWrite.addParam(*it_id, params)) {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID; 
        }
    }
    
    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

int DxlDriver::syncWrite2Bytes(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, DXL_LEN_TWO_BYTES);

    if (id_list.size() != data_list.size()) {
        return LEN_ID_DATA_NOT_SAME; 
    }
    
    if (id_list.size() == 0) {
        return COMM_SUCCESS;
    }
    
    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id=id_list.begin(), it_data=data_list.begin() ; 
            it_id < id_list.end() && it_data < data_list.end() ; 
            it_id++, it_data++) {
        uint8_t params[2] = { DXL_LOBYTE((uint16_t)(*it_data)), DXL_HIBYTE((uint16_t)(*it_data)) };
        if (!groupSyncWrite.addParam(*it_id, params)) {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }
    
    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

int DxlDriver::syncWrite4Bytes(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, DXL_LEN_FOUR_BYTES);

    if (id_list.size() != data_list.size()) {
        return LEN_ID_DATA_NOT_SAME; 
    }
    
    if (id_list.size() == 0) {
        return COMM_SUCCESS;
    }
    
    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id=id_list.begin(), it_data=data_list.begin() ; 
            it_id < id_list.end() && it_data < data_list.end() ; 
            it_id++, it_data++) {
        uint8_t params[4] = { DXL_LOBYTE(DXL_LOWORD(*it_data)), DXL_HIBYTE(DXL_LOWORD(*it_data)),
            DXL_LOBYTE(DXL_HIWORD(*it_data)), DXL_HIBYTE(DXL_HIWORD(*it_data)) };
        if (!groupSyncWrite.addParam(*it_id, params)) {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }
    
    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

/*
 *  -----------------   READ   --------------------
 */

int DxlDriver::read1Byte(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint8_t read_data;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0) {
        return dxl_error;
    }

    return dxl_comm_result;
}

int DxlDriver::read2Bytes(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint16_t read_data;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0) {
        return dxl_error;
    }

    return dxl_comm_result;
}

int DxlDriver::read4Bytes(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint32_t read_data;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0) {
        return dxl_error;
    }

    return dxl_comm_result;
}

/*
 *  -----------------   SYNC READ   --------------------
 */

int DxlDriver::syncRead(uint8_t address, uint8_t data_len, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    data_list.clear();

    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, address, data_len);
    bool dxl_getdata_result = false;
    int dxl_comm_result = COMM_TX_FAIL;

    std::vector<uint8_t>::iterator it_id;

    for (it_id=id_list.begin() ; it_id < id_list.end() ; it_id++) {
        if (!groupSyncRead.addParam(*it_id)) {
            groupSyncRead.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }
    
    dxl_comm_result = groupSyncRead.txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        groupSyncRead.clearParam();
        return dxl_comm_result;
    }

    for (it_id=id_list.begin() ; it_id < id_list.end() ; it_id++) {
        dxl_getdata_result = groupSyncRead.isAvailable(*it_id, address, data_len);
        if (!dxl_getdata_result) {
            groupSyncRead.clearParam();
            return GROUP_SYNC_READ_RX_FAIL;
        }
        if (data_len == DXL_LEN_ONE_BYTE) {
            data_list.push_back((uint8_t)groupSyncRead.getData(*it_id, address, data_len));
        }
        else if (data_len == DXL_LEN_TWO_BYTES) {
            data_list.push_back((uint16_t)groupSyncRead.getData(*it_id, address, data_len));
        }
        else if (data_len == DXL_LEN_FOUR_BYTES) {
            data_list.push_back((uint32_t)groupSyncRead.getData(*it_id, address, data_len));
        }
    }

    groupSyncRead.clearParam();
    return dxl_comm_result;

}
