/*
    change_hardware_version.cpp
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

#include "niryo_one_driver/change_hardware_version.h"

int change_hardware_version_and_reboot(int old_version, int new_version)
{
#ifdef __arm__
    
    std::ostringstream text;
    
    // get launch file path
    std::string folder_path = ros::package::getPath("niryo_one_bringup");
    std::string file_path = folder_path + "/launch/niryo_one_base.launch";

    std::ifstream in_file(file_path.c_str());

    // read launch file
    text << in_file.rdbuf();
    std::string str = text.str();
    if (str.size() == 0) {
        ROS_ERROR("Change hardware version : Could not open file %s", file_path.c_str());
        return CHANGE_HW_VERSION_FAIL;
    }

    // replace version in str
    std::string str_search  = "<arg name=\"hardware_version\" default=\"" + std::to_string(old_version);
    std::string str_replace = "<arg name=\"hardware_version\" default=\"" + std::to_string(new_version);
    
    size_t pos = str.find(str_search);
    if (pos == -1) {
        if (str.find(str_replace) != -1) {
            ROS_WARN("Change hardware version : Version is already correct (V%d)", new_version);
            return CHANGE_HW_VERSION_OK;
        }
        ROS_ERROR("Change hardware_version : Malformed niryo_one_base.launch, can't find hardware version");
        return CHANGE_HW_VERSION_FAIL;
    }
   
    try {
        str.replace(pos, str_search.length(), str_replace);
    }
    catch(const std::out_of_range& e) {
        ROS_INFO("Exception : %s", e.what());
    }

    // close launch file
    in_file.close();

    // re-write launch file
    std::ofstream out_file(file_path.c_str());
    out_file << str;

    ROS_INFO("Successfully changed hardware version in launch file (from V%d to V%d)", old_version, new_version);

    bool reboot;
    ros::param::get("/niryo_one/reboot_when_auto_change_version", reboot);

    if (reboot) {
        ROS_INFO("Reboot in 1 second...");
        std::system("sleep 1 && sudo reboot&");
    }

    return CHANGE_HW_VERSION_OK;
#endif

    // this function only works on Raspberry Pi 3 board.
    // On other computers, if you run Niryo One ROS packages, it means
    // that you are in simulation mode
    // --> if you want to change the hardware version (but you shouldn't have to)
    // you can do it manually on niryo_one_base.launch
    return CHANGE_HW_VERSION_NOT_ARM; 
}
