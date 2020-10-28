/*
    dxl_motor_state.h
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

#ifndef NIRYO_DXL_MOTOR_STATE_H
#define NIRYO_DXL_MOTOR_STATE_H

#include <string>

#define TOOL_STATE_PING_OK       0x01
#define TOOL_STATE_PING_ERROR    0x02
#define TOOL_STATE_WRONG_ID      0x03
#define TOOL_STATE_TIMEOUT       0x04

#define GRIPPER_STATE_OPEN       0x10 
#define GRIPPER_STATE_CLOSE      0x11

#define VACUUM_PUMP_STATE_PULLED 0x20
#define VACUUM_PUMP_STATE_PUSHED 0x21

#define MOTOR_TYPE_XL320 1
#define MOTOR_TYPE_XL430 2

struct DxlCustomCommand {
   
    DxlCustomCommand(int m, uint8_t i, uint32_t v, uint32_t r, uint32_t b)
        : motor_type(m), id(i), value(v), reg_address(r), byte_number(b) {}

    int motor_type;
    uint8_t id;
    uint32_t value;
    uint32_t reg_address;
    uint32_t byte_number;

};

class DxlMotorState {

    public:
        DxlMotorState() {}
        DxlMotorState(const std::string name, uint8_t id, int type, uint32_t init_position) {
            this->name = name;
            this->id = id;
            this->type = type;
            this->init_position = init_position;
            is_enabled = false;

            resetState();
            resetCommand();
        }

        void resetState() {
            state_pos = init_position;
            state_vel = 0;
            state_torque = 0;
            state_temperature = 0;
            state_voltage = 0;
            state_hw_error = 0;
        }
        void resetCommand() {
            cmd_pos = init_position;
            cmd_vel = 0;
            cmd_torque = 0;
            cmd_led = 0;
        }

        std::string getName()        { return name; }
        void setName(std::string n)  { name = n; } 
        uint8_t getId()              { return id; }
        void setId(uint8_t motor_id) { id = motor_id; } // allows to change tool motor easily
        int getType()                { return type; }
        void enable()                { is_enabled = true; }
        void disable()               { is_enabled = false; }
        bool isEnabled()             { return is_enabled; }
        
        // getters - state
        uint32_t getPositionState()      { return state_pos; }
        uint32_t getVelocityState()      { return state_vel; }
        uint32_t getTorqueState()        { return state_torque; }
        uint32_t getTemperatureState()   { return state_temperature; }
        uint32_t getVoltageState()       { return state_voltage; }
        uint32_t getHardwareErrorState() { return state_hw_error; }

        // setters - state
        void setPositionState(uint32_t pos)      { state_pos = pos; }
        void setVelocityState(uint32_t vel)      { state_vel = vel; }
        void setTorqueState(uint32_t torque)     { state_torque = torque; }
        void setTemperatureState(uint32_t temp)  { state_temperature = temp; }
        void setVoltageState(uint32_t volt)      { state_voltage = volt; }
        void setHardwareError(uint32_t hw_error) { state_hw_error = hw_error; }

        // getters - command
        uint32_t getPositionCommand() { return cmd_pos; }
        uint32_t getVelocityCommand() { return cmd_vel; }
        uint32_t getTorqueCommand()   { return cmd_torque; }
        uint32_t getLedCommand()      { return cmd_led; }

        // setters - command
        void setPositionCommand(uint32_t pos)  { cmd_pos = pos; }
        void setVelocityCommand(uint32_t vel)  { cmd_vel = vel; }
        void setTorqueCommand(uint32_t torque) { cmd_torque = torque; }
        void setLedCommand(uint32_t led)       { cmd_led = led; }

    private:

        std::string name;
        uint8_t id;
        int type;
        bool is_enabled;
        uint32_t init_position;

        // read variables
        
        uint32_t state_pos; 
        uint32_t state_vel;
        uint32_t state_torque;
        uint32_t state_temperature;
        uint32_t state_voltage;
        uint32_t state_hw_error;

        // write variables 

        uint32_t cmd_pos;
        uint32_t cmd_vel;
        uint32_t cmd_torque; 
        uint32_t cmd_led;
};

#endif
