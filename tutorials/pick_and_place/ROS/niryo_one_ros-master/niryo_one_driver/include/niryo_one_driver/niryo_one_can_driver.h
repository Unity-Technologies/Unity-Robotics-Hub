/*
    niryo_one_can_driver.h
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

#ifndef NIRYO_CAN_DRIVER_H
#define NIRYO_CAN_DRIVER_H

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include "mcp_can_rpi/mcp_can_rpi.h"

#define CAN_CMD_POSITION     0x03
#define CAN_CMD_TORQUE       0x04
#define CAN_CMD_MODE         0x07
#define CAN_CMD_MICRO_STEPS  0x13
#define CAN_CMD_OFFSET       0x14
#define CAN_CMD_CALIBRATE    0x15
#define CAN_CMD_SYNCHRONIZE  0x16
#define CAN_CMD_MAX_EFFORT   0x17
#define CAN_CMD_MOVE_REL     0x18
#define CAN_CMD_RESET        0x19 // not yet implemented

#define CAN_DATA_CONVEYOR_STATE 0x07
#define CAN_DATA_POSITION    0x03
#define CAN_DATA_DIAGNOSTICS 0x08
#define CAN_DATA_CALIBRATION_RESULT 0x09
#define CAN_DATA_FIRMWARE_VERSION 0x10

#define STEPPER_CONTROL_MODE_RELAX    0
#define STEPPER_CONTROL_MODE_STANDARD 1
#define STEPPER_CONVEYOR_OFF 20
#define STEPPER_CONVEYOR_ON 21
#define CAN_UPDATE_CONVEYOR_ID 23
#define STEPPER_CONTROL_MODE_PID_POS  2
#define STEPPER_CONTROL_MODE_TORQUE   3

#define CAN_MODEL_NUMBER 10000

class NiryoCanDriver
{
    private:

        boost::shared_ptr<MCP_CAN> mcp_can;

    public:

        NiryoCanDriver(int spi_channel, int spi_baudrate, INT8U gpio_can_interrupt);

        bool setupInterruptGpio();
        bool setupSpi();
        INT8U init();
        bool canReadData();
        INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U *buf);


        INT8U sendPositionCommand(int id, int cmd);
        INT8U sendRelativeMoveCommand(int id, int steps, int delay);
        INT8U sendTorqueOnCommand(int id, int torque_on);
        INT8U sendPositionOffsetCommand(int id, int cmd, int absolute_steps_at_offset_position);
        INT8U sendCalibrationCommand(int i, int offset, int delay, int direction, int timeout);
        INT8U sendSynchronizePositionCommand(int id, bool begin_traj);
        INT8U sendMicroStepsCommand(int id, int micro_steps);
        INT8U sendMaxEffortCommand(int id, int effort);
        // Conveyor functions
        INT8U sendConveyoOnCommand(int id, bool conveyor_on, int conveyor_speed, int8_t direction) ;
        INT8U sendUpdateConveyorId(uint8_t old_id, uint8_t new_id);

};

#endif
