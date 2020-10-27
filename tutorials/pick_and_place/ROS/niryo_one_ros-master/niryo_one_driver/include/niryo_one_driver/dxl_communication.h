/*
    dxl_communication.h
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
*/

#ifndef DXL_COMMUNICATION_H
#define DXL_COMMUNICATION_H

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "niryo_one_driver/dxl_motor_state.h"
#include "niryo_one_driver/xl320_driver.h"
#include "niryo_one_driver/xl430_driver.h"

#define DXL_MOTOR_4_ID   2 // V2 - axis 4
#define DXL_MOTOR_5_ID   3 // V2 - axis 5
#define DXL_MOTOR_5_1_ID 4 // V1 - axis 5 (1)
#define DXL_MOTOR_5_2_ID 5 // V1 - axis 5 (2)
#define DXL_MOTOR_6_ID   6

#define DXL_BUS_PROTOCOL_VERSION 2.0

#define DXL_FAIL_OPEN_PORT         -4500
#define DXL_FAIL_PORT_SET_BAUDRATE -4501
#define DXL_FAIL_SETUP_GPIO        -4502

// we stop at 1022 instead of 1023, to get an odd number of positions (1023)
// --> so we can get a middle point (511)
#define XL320_TOTAL_ANGLE          296.67
#define XL320_MAX_POSITION         1022
#define XL320_MIN_POSITION         0
#define XL320_MIDDLE_POSITION      511
#define XL320_TOTAL_RANGE_POSITION 1023

// we stop at 4094 instead of 4095, to get an odd number of positions (4095)
// --> so we can get a middle point (2047)
#define XL430_TOTAL_ANGLE          360.36
#define XL430_MAX_POSITION         4094
#define XL430_MIN_POSITION         0
#define XL430_MIDDLE_POSITION      2047
#define XL430_TOTAL_RANGE_POSITION 4095

#define RADIAN_TO_DEGREE 57.295779513082320876798154814105

#define TIME_TO_WAIT_IF_BUSY 0.0005

#define DXL_SCAN_OK                0
#define DXL_SCAN_MISSING_MOTOR    -50 
#define DXL_SCAN_UNALLOWED_MOTOR  -51

#define DXL_GRIPPER_ACTION_TIMEOUT 5

#define DXL_CONTROL_MODE_POSITION 1
#define DXL_CONTROL_MODE_VELOCITY 2
#define DXL_CONTROL_MODE_TORQUE   3

// according to xl-320 datasheet : 1 speed ~ 0.111 rpm ~ 1.8944 dxl position per second
#define XL320_STEPS_FOR_1_SPEED 1.8944 // 0.111 * 1024 / 60


class DxlCommunication {

    public:
        
        DxlCommunication();
        int init(int hardware_version);
        int setupCommunication();

        void startHardwareControlLoop(bool limited_mode);
        void stopHardwareControlLoop();

        void getCurrentPositionV1(double *axis_5_pos, double *axis_6_pos); 
        void getCurrentPositionV2(double *axis_4_pos, double *axis_5_pos, double *axis_6_pos); 
        
        void getHardwareStatus(bool *is_connection_ok, std::string &error_message,
                int *calibration_needed, bool *calibration_in_progress,
                std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
                std::vector<int32_t> &temperatures,
                std::vector<double> &voltages, std::vector<int32_t> &hw_errors);
        bool isConnectionOk();
        bool isOnLimitedMode();

        void setControlMode(int control_mode); // position, velocity, or torque
        void setGoalPositionV1(double axis_5_pos, double axis_6_pos);
        void setGoalPositionV2(double axis_4_pos, double axis_5_pos, double axis_6_pos);
        void setTorqueOn(bool on);
        void setLeds(std::vector<int> &leds);

        int scanAndCheck();
        int detectVersion();

        void moveAllMotorsToHomePosition();
        void addCustomDxlCommand(int motor_type, uint8_t id, uint32_t value,
                uint32_t reg_address, uint32_t byte_number);
        void rebootMotors();

        // Dxl Tools
        void setTool(uint8_t id, std::string name);
        int pingAndSetTool(uint8_t id, std::string name);
        
        int openGripper(uint8_t id, uint16_t open_position, uint16_t open_speed, uint16_t open_hold_torque);
        int closeGripper(uint8_t id, uint16_t close_position, uint16_t close_speed, uint16_t close_hold_torque, uint16_t close_max_torque);
        
        int pullAirVacuumPump(uint8_t id, uint16_t pull_air_position, uint16_t pull_air_hold_torque);
        int pushAirVacuumPump(uint8_t id, uint16_t push_air_position);

    private:

        // Niryo One hardware version
        int hardware_version;

        std::string device_name;
        int uart_baudrate;
        
        dynamixel::PortHandler *dxlPortHandler;
        dynamixel::PacketHandler *dxlPacketHandler;
       
        boost::shared_ptr<XL320Driver> xl320;
        boost::shared_ptr<XL430Driver> xl430;

        std::vector<uint8_t> required_motors_ids;
        std::vector<uint8_t> allowed_motors_ids;

        uint32_t rad_pos_to_xl320_pos(double position_rad);
        double   xl320_pos_to_rad_pos(uint32_t position_dxl);

        uint32_t rad_pos_to_xl430_pos(double position_rad);
        double   xl430_pos_to_rad_pos(uint32_t position_dxl);

        void hardwareControlLoop();
        void hardwareControlRead();
        void hardwareControlWrite();

        void resetHardwareControlLoopRates();

        boost::shared_ptr<std::thread> hardware_control_loop_thread;

        // motors 
        DxlMotorState m4; // V2 only
        DxlMotorState m5; // V2 only
        DxlMotorState m5_1; // V1 only
        DxlMotorState m5_2; // V1 only
        DxlMotorState m6; // V1 + V2
        DxlMotorState tool; // V1 + V2
        std::vector<DxlMotorState*> motors;

        // for hardware control
        
        bool is_dxl_connection_ok;
        std::string debug_error_message;

        uint8_t torque_on; // torque is ON/OFF for all motors at the same time
        
        bool is_tool_connected;

        bool hw_control_loop_keep_alive;
        bool hw_is_busy;
        bool hw_limited_mode;

        double hw_control_loop_frequency;

        int xl320_hw_fail_counter_read;
        int xl430_hw_fail_counter_read;

        double time_hw_data_last_write;
        double time_hw_data_last_read;
        double time_hw_status_last_read;
        double hw_data_write_frequency;
        double hw_data_read_frequency;
        double hw_status_read_frequency;

        std::queue<DxlCustomCommand> custom_command_queue;
        bool should_reboot_motors;

        // enable flags

        bool read_position_enable;
        bool read_velocity_enable;
        bool read_torque_enable;
        bool read_hw_status_enable; // for temperature + voltage + hw_error

        bool write_position_enable;
        bool write_velocity_enable;
        bool write_torque_enable;
        bool write_led_enable;
        bool write_torque_on_enable;
        bool write_tool_enable;
};


#endif
