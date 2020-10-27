/*
    motor_offset_file_handler.h
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

#ifndef MOTOR_OFFSET_FILE_HANDLER_H
#define MOTOR_OFFSET_FILE_HANDLER_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <unistd.h>

bool get_motors_calibration_offsets(std::vector<int> &motor_id_list,  std::vector<int> &steps_list);

bool set_motors_calibration_offsets(std::vector<int> &motor_id_list,  std::vector<int> &steps_list);

#endif
