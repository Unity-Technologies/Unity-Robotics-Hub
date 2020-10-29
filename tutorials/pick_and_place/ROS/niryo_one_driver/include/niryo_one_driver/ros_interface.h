/*
    ros_interface.h
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

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include <thread>

#include <ros/ros.h>

#include "niryo_one_driver/communication_base.h"
#include "niryo_one_driver/rpi_diagnostics.h"
#include "niryo_one_driver/change_hardware_version.h"

#include "niryo_one_msgs/SetInt.h"
#include "niryo_one_msgs/SetLeds.h"

#include "niryo_one_msgs/PingDxlTool.h"
#include "niryo_one_msgs/OpenGripper.h"
#include "niryo_one_msgs/CloseGripper.h"
#include "niryo_one_msgs/PullAirVacuumPump.h"
#include "niryo_one_msgs/PushAirVacuumPump.h"

#include "niryo_one_msgs/ChangeHardwareVersion.h"
#include "niryo_one_msgs/SendCustomDxlValue.h"
#include "niryo_one_msgs/SetConveyor.h"
#include "niryo_one_msgs/ControlConveyor.h"
#include "niryo_one_msgs/UpdateConveyorId.h"

#include "niryo_one_msgs/HardwareStatus.h"
#include "niryo_one_msgs/SoftwareVersion.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8MultiArray.h"
#include "niryo_one_msgs/ConveyorFeedback.h"

class RosInterface {

    public:

        RosInterface(CommunicationBase* niryo_one_comm, RpiDiagnostics* rpi_diagnostics,
                bool *flag_reset_controllers, bool learning_mode_on, int hardware_version);

        void startServiceServers();
        void startPublishers();
        void startSubscribers();

    private:

        CommunicationBase* comm;
        RpiDiagnostics* rpi_diagnostics;
        ros::NodeHandle nh_;

        bool* flag_reset_controllers;
        int hardware_version;
        bool learning_mode_on;
        int calibration_needed;
        bool last_connection_up_flag;

        std::string rpi_image_version;
        std::string ros_niryo_one_version;

        // publishers

        ros::Publisher hardware_status_publisher;
        boost::shared_ptr<std::thread> publish_hardware_status_thread;

        ros::Publisher software_version_publisher;
        boost::shared_ptr<std::thread> publish_software_version_thread;

        ros::Publisher learning_mode_publisher;
        boost::shared_ptr<std::thread> publish_learning_mode_thread;


        ros::Publisher conveyor_status_publisher;
        boost::shared_ptr<std::thread> publish_conveyor_status_thread;
        
        ros::Publisher conveyor_1_feedback_publisher;
        boost::shared_ptr<std::thread> publish_conveyor_1_feedback_thread;

        ros::Publisher conveyor_2_feedback_publisher;
        boost::shared_ptr<std::thread> publish_conveyor_2_feedback_thread;

        // publish methods

        void publishHardwareStatus();
        void publishSoftwareVersion();
        void publishLearningMode(); 
        void publishConveyor1Feedback();
        void publishConveyor2Feedback(); 

        // services

        ros::ServiceServer calibrate_motors_server;
        ros::ServiceServer request_new_calibration_server;

        ros::ServiceServer activate_learning_mode_server;
        ros::ServiceServer activate_leds_server;

        ros::ServiceServer ping_and_set_dxl_tool_server;

        ros::ServiceServer open_gripper_server;
        ros::ServiceServer close_gripper_server;
        ros::ServiceServer pull_air_vacuum_pump_server;
        ros::ServiceServer push_air_vacuum_pump_server;

        ros::ServiceServer change_hardware_version_server;
        ros::ServiceServer send_custom_dxl_value_server;
        ros::ServiceServer reboot_motors_server;

        // Conveyor services
        ros::ServiceServer ping_and_set_stepper_server;
        ros::ServiceServer control_conveyor_server;
        ros::ServiceServer update_conveyor_id_server;

        // callbacks

        bool callbackCalibrateMotors(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res);
        bool callbackRequestNewCalibration(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res);

        bool callbackActivateLearningMode(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res);
        bool callbackActivateLeds(niryo_one_msgs::SetLeds::Request &req, niryo_one_msgs::SetLeds::Response &res);

        bool callbackPingAndSetDxlTool(niryo_one_msgs::PingDxlTool::Request &req, niryo_one_msgs::PingDxlTool::Response &res);

        bool callbackPingAndSetConveyor(niryo_one_msgs::SetConveyor::Request &req, niryo_one_msgs::SetConveyor::Response &res);
        bool callbackControlConveyor(niryo_one_msgs::ControlConveyor::Request &req, niryo_one_msgs::ControlConveyor::Response &res);
        bool callbackUpdateIdConveyor(niryo_one_msgs::UpdateConveyorId::Request &req, niryo_one_msgs::UpdateConveyorId::Response &res);

        bool callbackOpenGripper(niryo_one_msgs::OpenGripper::Request &req, niryo_one_msgs::OpenGripper::Response &res);
        bool callbackCloseGripper(niryo_one_msgs::CloseGripper::Request &req, niryo_one_msgs::CloseGripper::Response &res);

        bool callbackPullAirVacuumPump(niryo_one_msgs::PullAirVacuumPump::Request &req, niryo_one_msgs::PullAirVacuumPump::Response &res);
        bool callbackPushAirVacuumPump(niryo_one_msgs::PushAirVacuumPump::Request &req, niryo_one_msgs::PushAirVacuumPump::Response &res);

        bool callbackChangeHardwareVersion(niryo_one_msgs::ChangeHardwareVersion::Request &req,
                niryo_one_msgs::ChangeHardwareVersion::Response &res);

        bool callbackSendCustomDxlValue(niryo_one_msgs::SendCustomDxlValue::Request &req, 
                niryo_one_msgs::SendCustomDxlValue::Response &res);

        bool callbackRebootMotors(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res);

};

#endif
