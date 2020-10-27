/*
    fake_communication.h
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

#ifndef FAKE_COMMUNICATION_H
#define FAKE_COMMUNICATION_H

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>

#include "niryo_one_driver/communication_base.h"

#include "niryo_one_driver/dxl_motor_state.h" // for gripper enums

class FakeCommunication : public CommunicationBase {

    public:
        FakeCommunication(int hardware_version);
        int init();

        void manageHardwareConnection();
        bool isConnectionOk();

        void startHardwareControlLoop();
        void stopHardwareControlLoop();
        void resumeHardwareControlLoop();

        void getCurrentPosition(double pos[6]);
        
        void getHardwareStatus(bool *is_connection_ok, std::string &error_message,
                int *calibration_needed, bool *calibration_in_progress,
                std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
                std::vector<int32_t> &temperatures,
                std::vector<double> &voltages, std::vector<int32_t> &hw_errors);
        
        void getFirmwareVersions(std::vector<std::string> &motor_names,
                std::vector<std::string> &firmware_versions);
        
        void sendPositionToRobot(const double cmd[6]); 
        void activateLearningMode(bool activate);
        bool setLeds(std::vector<int> &leds, std::string &message);
        
        int allowMotorsCalibrationToStart(int mode, std::string &result_message);
        void requestNewCalibration();
        bool isCalibrationInProgress();
        
        // tools
        int pingAndSetDxlTool(uint8_t id, std::string name);
        
        int openGripper(uint8_t id, uint16_t open_position, uint16_t open_speed, uint16_t open_hold_torque);
        int closeGripper(uint8_t id, uint16_t close_position, uint16_t close_speed, uint16_t close_hold_torque, uint16_t close_max_torque);
        
        int pullAirVacuumPump(uint8_t id, uint16_t pull_air_position, uint16_t pull_air_hold_torque);
        int pushAirVacuumPump(uint8_t id, uint16_t push_air_position);
        
        // steppers
        void synchronizeMotors(bool begin_traj);
        
        void addCustomDxlCommand(int motor_type, uint8_t id, uint32_t value,
                uint32_t reg_address, uint32_t byte_number);

        void rebootMotors();
        // conveyor belt
        int pingAndSetConveyor(uint8_t id, bool activate, std::string &message);
        int moveConveyor(uint8_t id, bool activate, int16_t speed, int8_t direction, std::string &message);
        int updateIdConveyor(uint8_t old_id, uint8_t new_id, std::string &message);
        void geCurrentConveyorStatus(int* status_conveyor_1 ,int* status_conveyor_2); 
        void getConveyorFeedBack(uint8_t conveyor_id, bool* connection_state, bool* running, int16_t* speed, int8_t* direction);

    private:

        int hardware_version;
        
        double echo_pos[6]; // just store cmd in this array, and echo position

};

#endif
