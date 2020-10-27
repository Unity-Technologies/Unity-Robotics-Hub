/*
    niryo_one_hardware_interface.cpp
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

#include "niryo_one_driver/niryo_one_hardware_interface.h"

NiryoOneHardwareInterface::NiryoOneHardwareInterface(CommunicationBase* niryo_one_comm) 
{
    comm = niryo_one_comm;
    ROS_INFO("Starting NiryoOne Hardware Interface...");

    // connect and register joint state interface
    hardware_interface::JointStateHandle state_handle1("joint_1", &pos[0], &vel[0], &eff[0]);
    joint_state_interface.registerHandle(state_handle1);
    hardware_interface::JointStateHandle state_handle2("joint_2", &pos[1], &vel[1], &eff[1]);
    joint_state_interface.registerHandle(state_handle2);
    hardware_interface::JointStateHandle state_handle3("joint_3", &pos[2], &vel[2], &eff[2]);
    joint_state_interface.registerHandle(state_handle3);
    hardware_interface::JointStateHandle state_handle4("joint_4", &pos[3], &vel[3], &eff[3]);
    joint_state_interface.registerHandle(state_handle4);
    hardware_interface::JointStateHandle state_handle5("joint_5", &pos[4], &vel[4], &eff[4]);
    joint_state_interface.registerHandle(state_handle5);
    hardware_interface::JointStateHandle state_handle6("joint_6", &pos[5], &vel[5], &eff[5]);
    joint_state_interface.registerHandle(state_handle6);

    registerInterface(&joint_state_interface);

    // connect and register joint position interface
    hardware_interface::JointHandle position_handle1(joint_state_interface.getHandle("joint_1"), &cmd[0]);
    joint_position_interface.registerHandle(position_handle1);
    hardware_interface::JointHandle position_handle2(joint_state_interface.getHandle("joint_2"), &cmd[1]);
    joint_position_interface.registerHandle(position_handle2);
    hardware_interface::JointHandle position_handle3(joint_state_interface.getHandle("joint_3"), &cmd[2]);
    joint_position_interface.registerHandle(position_handle3);
    hardware_interface::JointHandle position_handle4(joint_state_interface.getHandle("joint_4"), &cmd[3]);
    joint_position_interface.registerHandle(position_handle4);
    hardware_interface::JointHandle position_handle5(joint_state_interface.getHandle("joint_5"), &cmd[4]);
    joint_position_interface.registerHandle(position_handle5);
    hardware_interface::JointHandle position_handle6(joint_state_interface.getHandle("joint_6"), &cmd[5]);
    joint_position_interface.registerHandle(position_handle6);

    registerInterface(&joint_position_interface);

    ROS_INFO("Interfaces registered.");
}

void NiryoOneHardwareInterface::setCommandToCurrentPosition()
{
    joint_position_interface.getHandle("joint_1").setCommand(pos[0]);
    joint_position_interface.getHandle("joint_2").setCommand(pos[1]);
    joint_position_interface.getHandle("joint_3").setCommand(pos[2]);
    joint_position_interface.getHandle("joint_4").setCommand(pos[3]);
    joint_position_interface.getHandle("joint_5").setCommand(pos[4]);
    joint_position_interface.getHandle("joint_6").setCommand(pos[5]);
}
                                          

void NiryoOneHardwareInterface::read()
{
    //ROS_INFO("Read sensor values");
    
    double pos_to_read[6] = {0.0};
    
    comm->getCurrentPosition(pos_to_read);
   
    pos[0] = pos_to_read[0];
    pos[1] = pos_to_read[1];
    pos[2] = pos_to_read[2];
    pos[3] = pos_to_read[3];
    pos[4] = pos_to_read[4];
    pos[5] = pos_to_read[5];
}

void NiryoOneHardwareInterface::write()
{
    // for debugging
    //pos[0] = cmd[0];
    //pos[1] = cmd[1];
    //pos[2] = cmd[2];
    //pos[3] = cmd[3];
    //pos[4] = cmd[4];
    //pos[5] = cmd[5];

    comm->sendPositionToRobot(cmd);
}
