/*
    xl320_driver.h
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

#ifndef XL320_DRIVER_H
#define XL320_DRIVER_H

#include "niryo_one_driver/dxl_driver.h"
#include <vector>

#define XL320_PROTOCOL_VERSION 2.0
#define XL320_MODEL_NUMBER 350

// Table here : http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
#define XL320_ADDR_MODEL_NUMBER          0
#define XL320_ADDR_FIRMWARE_VERSION      2
#define XL320_ADDR_ID                    3
#define XL320_ADDR_BAUDRATE              4
#define XL320_ADDR_RETURN_DELAY_TIME     5
#define XL320_ADDR_CW_ANGLE_LIMIT        6  
#define XL320_ADDR_CCW_ANGLE_LIMIT       8                  // EEPROM
#define XL320_ADDR_CONTROL_MODE          11
#define XL320_ADDR_LIMIT_TEMPERATURE     12
#define XL320_ADDR_LOWER_LIMIT_VOLTAGE   13
#define XL320_ADDR_UPPER_LIMIT_VOLTAGE   14
#define XL320_ADDR_MAX_TORQUE            15
#define XL320_ADDR_RETURN_LEVEL          17
#define XL320_ADDR_ALARM_SHUTDOWN        18

#define XL320_ADDR_TORQUE_ENABLE         24
#define XL320_ADDR_LED                   25
#define XL320_ADDR_D_GAIN                27
#define XL320_ADDR_I_GAIN                28
#define XL320_ADDR_P_GAIN                29
#define XL320_ADDR_GOAL_POSITION         30
#define XL320_ADDR_GOAL_SPEED            32
#define XL320_ADDR_GOAL_TORQUE           35
#define XL320_ADDR_PRESENT_POSITION      37                  // RAM
#define XL320_ADDR_PRESENT_SPEED         39
#define XL320_ADDR_PRESENT_LOAD          41
#define XL320_ADDR_PRESENT_VOLTAGE       45
#define XL320_ADDR_PRESENT_TEMPERATURE   46
#define XL320_ADDR_REGISTERED            47
#define XL320_ADDR_MOVING                49
#define XL320_ADDR_HW_ERROR_STATUS       50                  
#define XL320_ADDR_PUNCH                 51

class XL320Driver : public DxlDriver {

    public:
        XL320Driver(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler);

        int checkModelNumber(uint8_t id);

        // eeprom write
        int changeId            (uint8_t id, uint8_t new_id);
        int changeBaudRate      (uint8_t id, uint32_t new_baudrate);
        int setReturnDelayTime  (uint8_t id, uint32_t return_delay_time);
        int setLimitTemperature (uint8_t id, uint32_t temperature);
        int setMaxTorque        (uint8_t id, uint32_t torque);
        int setReturnLevel      (uint8_t id, uint32_t return_level);
        int setAlarmShutdown    (uint8_t id, uint32_t alarm_shutdown);

        // eeprom read
        int readReturnDelayTime  (uint8_t id, uint32_t *return_delay_time);
        int readLimitTemperature (uint8_t id, uint32_t *limit_temperature);
        int readMaxTorque        (uint8_t id, uint32_t *max_torque);
        int readReturnLevel      (uint8_t id, uint32_t *return_level);
        int readAlarmShutdown    (uint8_t id, uint32_t *alarm_shutdown);

        // ram write
        int setTorqueEnable   (uint8_t id, uint32_t torque_enable);
        int setLed            (uint8_t id, uint32_t led_value);
        int setGoalPosition   (uint8_t id, uint32_t position);
        int setGoalVelocity   (uint8_t id, uint32_t velocity);
        int setGoalTorque     (uint8_t id, uint32_t torque);

        int syncWriteLed          (std::vector<uint8_t> &id_list, std::vector<uint32_t> &led_list);
        int syncWriteTorqueEnable (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_enable_list); 
        int syncWritePositionGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list);
        int syncWriteVelocityGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list);
        int syncWriteTorqueGoal   (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_list);

        // ram read
        int readPosition       (uint8_t id, uint32_t *present_position);
        int readVelocity       (uint8_t id, uint32_t *present_velocity);
        int readLoad           (uint8_t id, uint32_t *present_load);
        int readTemperature    (uint8_t id, uint32_t *temperature);
        int readVoltage        (uint8_t id, uint32_t *voltage);
        int readHardwareStatus (uint8_t id, uint32_t *hardware_status);

        int syncReadPosition       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list);
        int syncReadVelocity       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list);
        int syncReadLoad           (std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list);
        int syncReadTemperature    (std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list);
        int syncReadVoltage        (std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list);
        int syncReadHwErrorStatus  (std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list);

        // custom write
        int customWrite(uint8_t id, uint32_t value, uint8_t reg_address, uint8_t byte_number);
};

#endif
