/*
    dxl_driver.h
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


#ifndef DXL_DRIVER_H
#define DXL_DRIVER_H

/*
    Base class for Dynamixel motor driver (dynamixel protocol 2.0 only)
*/

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <vector>

#define DXL_LEN_ONE_BYTE   1
#define DXL_LEN_TWO_BYTES  2
#define DXL_LEN_FOUR_BYTES 4

#define GROUP_SYNC_REDONDANT_ID     10
#define GROUP_SYNC_READ_RX_FAIL     11
#define LEN_ID_DATA_NOT_SAME        20

#define PING_WRONG_MODEL_NUMBER     30

// Communication Result (dynamixel_sdk)
//#define COMM_SUCCESS        0       // tx or rx packet communication success
//#define COMM_PORT_BUSY      -1000   // Port is busy (in use)                    
//#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet      
//#define COMM_RX_FAIL        -1002   // Failed get status packet               
//#define COMM_TX_ERROR       -2000   // Incorrect instruction packet               FROM dynamixel_sdk
//#define COMM_RX_WAITING     -3000   // Now recieving status packet           
//#define COMM_RX_TIMEOUT     -3001   // There is no status packet            
//#define COMM_RX_CORRUPT     -3002   // Incorrect status packet             
//#define COMM_NOT_AVAILABLE  -9000 //                                      
//
/////////////////// Protocol 2.0 Error bit /////////////////
//#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.  
//#define ERRNUM_INSTRUCTION      2       // Instruction error                          
//#define ERRNUM_CRC              3       // CRC check error                            
//#define ERRNUM_DATA_RANGE       4       // Data range error                           
//#define ERRNUM_DATA_LENGTH      5       // Data length error                          
//#define ERRNUM_DATA_LIMIT       6       // Data limit error                           
//#define ERRNUM_ACCESS 7 // Access error                                              

//#define ERRBIT_ALERT 128 //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

class DxlDriver {

    protected:
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        int syncWrite1Byte  (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);
        int syncWrite2Bytes (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);
        int syncWrite4Bytes (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);

        int read1Byte       (uint8_t address, uint8_t id, uint32_t *data);
        int read2Bytes      (uint8_t address, uint8_t id, uint32_t *data);
        int read4Bytes      (uint8_t address, uint8_t id, uint32_t *data);
        int syncRead        (uint8_t address, uint8_t data_len, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);

    public:
        DxlDriver(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler);
        
        int scan(std::vector<uint8_t> &id_list);
        int ping(uint8_t id);
        int getModelNumber(uint8_t id, uint16_t *dxl_model_number);
        int reboot(uint8_t id);

        /*
         * Virtual functions below - to override
         *
         * --> All functions return communication result
         */

        virtual int checkModelNumber(uint8_t id) = 0;

        // eeprom write
        virtual int changeId            (uint8_t id, uint8_t new_id) = 0;
        virtual int changeBaudRate      (uint8_t id, uint32_t new_baudrate) = 0;
        virtual int setReturnDelayTime  (uint8_t id, uint32_t return_delay_time) = 0;
        virtual int setLimitTemperature (uint8_t id, uint32_t temperature) = 0;
        virtual int setMaxTorque        (uint8_t id, uint32_t torque) = 0;
        virtual int setReturnLevel      (uint8_t id, uint32_t return_level) = 0;
        virtual int setAlarmShutdown    (uint8_t id, uint32_t alarm_shutdown) = 0;

        // eeprom read
        virtual int readReturnDelayTime  (uint8_t id, uint32_t *return_delay_time) = 0;
        virtual int readLimitTemperature (uint8_t id, uint32_t *limit_temperature) = 0;
        virtual int readMaxTorque        (uint8_t id, uint32_t *max_torque) = 0;
        virtual int readReturnLevel      (uint8_t id, uint32_t *return_level) = 0;
        virtual int readAlarmShutdown    (uint8_t id, uint32_t *alarm_shutdown) = 0;

        // ram write
        virtual int setTorqueEnable   (uint8_t id, uint32_t torque_enable) = 0;
        virtual int setLed            (uint8_t id, uint32_t led_value) = 0;
        virtual int setGoalPosition   (uint8_t id, uint32_t position) = 0;
        virtual int setGoalVelocity   (uint8_t id, uint32_t velocity) = 0;
        virtual int setGoalTorque     (uint8_t id, uint32_t torque) = 0;

        virtual int syncWriteLed          (std::vector<uint8_t> &id_list, std::vector<uint32_t> &led_list) = 0;
        virtual int syncWriteTorqueEnable (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_enable_list) = 0; 
        virtual int syncWritePositionGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) = 0;
        virtual int syncWriteVelocityGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) = 0;
        virtual int syncWriteTorqueGoal   (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_list) = 0;

        // ram read
        virtual int readPosition       (uint8_t id, uint32_t *present_position) = 0;
        virtual int readVelocity       (uint8_t id, uint32_t *present_velocity) = 0;
        virtual int readLoad           (uint8_t id, uint32_t *present_load) = 0;
        virtual int readTemperature    (uint8_t id, uint32_t *temperature) = 0;
        virtual int readVoltage        (uint8_t id, uint32_t *voltage) = 0;
        virtual int readHardwareStatus (uint8_t id, uint32_t *hardware_status) = 0;

        virtual int syncReadPosition       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) = 0;
        virtual int syncReadVelocity       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) = 0;
        virtual int syncReadLoad           (std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) = 0;
        virtual int syncReadTemperature    (std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) = 0;
        virtual int syncReadVoltage        (std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) = 0;
        virtual int syncReadHwErrorStatus  (std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) = 0;
};

#endif
