/*
    ros_interface.cpp
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

#include "niryo_one_driver/ros_interface.h"

RosInterface::RosInterface(CommunicationBase* niryo_one_comm, RpiDiagnostics* rpi_diagnostics,
        bool *flag_reset_controllers, bool learning_mode_on, int hardware_version)
{
    comm = niryo_one_comm;
    this->rpi_diagnostics = rpi_diagnostics;
    this->learning_mode_on = learning_mode_on;
    this->flag_reset_controllers = flag_reset_controllers;
    this->hardware_version = hardware_version;
    last_connection_up_flag = true;
    
    ros::param::get("/niryo_one/info/image_version", rpi_image_version);
    ros::param::get("/niryo_one/info/ros_version", ros_niryo_one_version);
   
    // trim string
    rpi_image_version.erase(rpi_image_version.find_last_not_of(" \n\r\t")+1);
    ros_niryo_one_version.erase(ros_niryo_one_version.find_last_not_of(" \n\r\t")+1);
    
    ROS_INFO("Ros interface started.");

    startServiceServers();
    startPublishers();

    // this flag is used to know if learning mode can be deactivated
    calibration_needed = 0;
}

bool RosInterface::callbackCalibrateMotors(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res) 
{
    int calibration_mode = req.value; 
    std::string result_message = "";
    int result = comm->allowMotorsCalibrationToStart(calibration_mode, result_message);

    res.status = result;
    res.message = result_message;

    // special case here 
    // we set flag learning_mode_on, but we don't activate from here
    // learning_mode should be activated in comm, AFTER motors have been calibrated
    // --> this fixes an issue where motors will jump back to a previous cmd after being calibrated
    learning_mode_on = true;

    return true;
}

bool RosInterface::callbackRequestNewCalibration(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res)
{
    // 1. Activate learning mode
    learning_mode_on = true;
    
    comm->activateLearningMode(learning_mode_on);
    
    // publish one time
    std_msgs::Bool msg;
    msg.data = learning_mode_on;
    learning_mode_publisher.publish(msg);

    // 2. Set calibration flag (user will have to validate for calibration to start)
    comm->requestNewCalibration();

    res.status = 200;
    res.message = "New calibration request has been made, you will be requested to confirm it.";
    return true;
}

/*
 * Deactivating learning mode (= activating motors) is possible only if motors are calibrated
 * Activating learning mode is also possible when waiting for calibration
 */
bool RosInterface::callbackActivateLearningMode(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res)
{
    if (comm->isCalibrationInProgress()) {
        res.status = 400;
        res.message = "You can't activate/deactivate learning mode during motors calibration";
        return true;
    }

    if (calibration_needed == 1 || !comm->isConnectionOk()) { // if can or dxl is disconnected, only allow to activate learning mode
        learning_mode_on = true;
    }
    else {
        learning_mode_on = req.value; 
    }
    
    // reset controller if learning mode -> OFF
    // we want motors to start where they physically are, not from the last command
    if (!learning_mode_on) {
        *(flag_reset_controllers) = true;
        ros::Duration(0.05).sleep();
    }
    
    comm->activateLearningMode(learning_mode_on);
    
    // publish one time
    std_msgs::Bool msg;
    msg.data = learning_mode_on;
    learning_mode_publisher.publish(msg);
   
    res.status = 200;
    res.message = (learning_mode_on) ? "Activating learning mode" : "Deactivating learning mode";
    return true;
}

bool RosInterface::callbackActivateLeds(niryo_one_msgs::SetLeds::Request &req, niryo_one_msgs::SetLeds::Response &res)
{
    std::vector<int> leds = req.values;
    std::string message = "";
    bool result = comm->setLeds(leds, message);

    res.status = (result) ? 200 : 400;
    res.message = message;
    return true;
}

bool RosInterface::callbackPingAndSetDxlTool(niryo_one_msgs::PingDxlTool::Request &req, niryo_one_msgs::PingDxlTool::Response &res)
{
    res.state = comm->pingAndSetDxlTool(req.id, req.name);
    return true;
}

bool RosInterface::callbackPingAndSetConveyor(niryo_one_msgs::SetConveyor::Request &req, niryo_one_msgs::SetConveyor::Response &res) {
    std::string message = "";
    res.status = comm->pingAndSetConveyor(req.id, req.activate, message);
    res.message = message; 
    return true;
}
bool RosInterface::callbackControlConveyor(niryo_one_msgs::ControlConveyor::Request &req, niryo_one_msgs::ControlConveyor::Response &res){
    std::string message = "";
    res.status = comm->moveConveyor(req.id, req.control_on, req.speed, req.direction, message);
    res.message = message; 
    return true;
}
bool  RosInterface::callbackUpdateIdConveyor(niryo_one_msgs::UpdateConveyorId::Request &req, niryo_one_msgs::UpdateConveyorId::Response &res){
    std::string message = "";
    res.status = comm->updateIdConveyor(req.old_id, req.new_id, message);
    res.message = message;
    return true;
}

bool RosInterface::callbackOpenGripper(niryo_one_msgs::OpenGripper::Request &req, niryo_one_msgs::OpenGripper::Response &res)
{
    res.state = comm->openGripper(req.id, req.open_position, req.open_speed, req.open_hold_torque);
    return true;
}

bool RosInterface::callbackCloseGripper(niryo_one_msgs::CloseGripper::Request &req, niryo_one_msgs::CloseGripper::Response &res)
{
    res.state = comm->closeGripper(req.id, req.close_position, req.close_speed, req.close_hold_torque, req.close_max_torque);
    return true;
}

bool RosInterface::callbackPullAirVacuumPump(niryo_one_msgs::PullAirVacuumPump::Request &req, niryo_one_msgs::PullAirVacuumPump::Response &res)
{
    res.state = comm->pullAirVacuumPump(req.id, req.pull_air_position, req.pull_air_hold_torque);
    return true;
}

bool RosInterface::callbackPushAirVacuumPump(niryo_one_msgs::PushAirVacuumPump::Request &req, niryo_one_msgs::PushAirVacuumPump::Response &res)
{
    res.state = comm->pushAirVacuumPump(req.id, req.push_air_position);
    return true;
}

bool RosInterface::callbackChangeHardwareVersion(niryo_one_msgs::ChangeHardwareVersion::Request &req,
        niryo_one_msgs::ChangeHardwareVersion::Response &res)
{
    int result = change_hardware_version_and_reboot(req.old_version, req.new_version);
    if (result == CHANGE_HW_VERSION_OK) {
        res.status = 200;
        res.message = "Successfully changed hardware version.";
    }
    else if (result == CHANGE_HW_VERSION_FAIL) {
        res.status = 400;
        res.message = "Failed to change hardware version, please check the ROS logs";
    }
    else if (result == CHANGE_HW_VERSION_NOT_ARM) {
        res.status = 400;
        res.message = "Not allowed to change hardware version on non-ARM system";
    }
    return true;
}

bool RosInterface::callbackSendCustomDxlValue(niryo_one_msgs::SendCustomDxlValue::Request &req,
        niryo_one_msgs::SendCustomDxlValue::Response &res)
{
    // pre-check motor type
    if (req.motor_type != 1 && req.motor_type != 2) {
        res.status = 400;
        res.message = "Invalid motor type: should be 1 (XL-320) or 2 (XL-430)";
        return true;
    }

    comm->addCustomDxlCommand(req.motor_type, req.id, req.value, req.reg_address, req.byte_number);

    res.status = 200;
    res.message = "OK";
    return true;
}

bool RosInterface::callbackRebootMotors(niryo_one_msgs::SetInt::Request &req, niryo_one_msgs::SetInt::Response &res)
{
    comm->rebootMotors();
    res.status = 200;
    res.message = "OK";
    return true;
}

void RosInterface::startServiceServers()
{
    calibrate_motors_server = nh_.advertiseService("niryo_one/calibrate_motors", &RosInterface::callbackCalibrateMotors, this);
    request_new_calibration_server = nh_.advertiseService("niryo_one/request_new_calibration", &RosInterface::callbackRequestNewCalibration, this);

    activate_learning_mode_server = nh_.advertiseService("niryo_one/activate_learning_mode", &RosInterface::callbackActivateLearningMode, this);
    activate_leds_server = nh_.advertiseService("niryo_one/set_dxl_leds", &RosInterface::callbackActivateLeds, this);

    ping_and_set_dxl_tool_server = nh_.advertiseService("niryo_one/tools/ping_and_set_dxl_tool", &RosInterface::callbackPingAndSetDxlTool, this);

    // steppers service test
    ping_and_set_stepper_server = nh_.advertiseService("niryo_one/kits/ping_and_set_conveyor", &RosInterface::callbackPingAndSetConveyor, this);
    control_conveyor_server = nh_.advertiseService("niryo_one/kits/control_conveyor", &RosInterface::callbackControlConveyor, this);
    update_conveyor_id_server = nh_.advertiseService("niryo_one/kits/update_conveyor_id", &RosInterface::callbackUpdateIdConveyor, this);

    open_gripper_server = nh_.advertiseService("niryo_one/tools/open_gripper", &RosInterface::callbackOpenGripper, this);
    close_gripper_server = nh_.advertiseService("niryo_one/tools/close_gripper", &RosInterface::callbackCloseGripper, this);
    pull_air_vacuum_pump_server = nh_.advertiseService("niryo_one/tools/pull_air_vacuum_pump", &RosInterface::callbackPullAirVacuumPump, this);
    push_air_vacuum_pump_server = nh_.advertiseService("niryo_one/tools/push_air_vacuum_pump", &RosInterface::callbackPushAirVacuumPump, this);

    change_hardware_version_server = nh_.advertiseService("niryo_one/change_hardware_version", &RosInterface::callbackChangeHardwareVersion, this);
    send_custom_dxl_value_server = nh_.advertiseService("niryo_one/send_custom_dxl_value", &RosInterface::callbackSendCustomDxlValue, this);
    reboot_motors_server = nh_.advertiseService("niryo_one/reboot_motors", &RosInterface::callbackRebootMotors, this);
}

void RosInterface::publishHardwareStatus()
{
    double publish_hw_status_frequency;
    ros::param::get("~publish_hw_status_frequency", publish_hw_status_frequency);
    ros::Rate publish_hardware_status_rate = ros::Rate(publish_hw_status_frequency);

    while (ros::ok()) {
        ros::Time time_now = ros::Time::now();

        bool connection_up = false;
        bool calibration_in_progress = false;
        std::string error_message;
        std::vector<std::string> motor_names;
        std::vector<std::string> motor_types;
        std::vector<int32_t> temperatures;
        std::vector<double> voltages;
        std::vector<int32_t> hw_errors;

        comm->getHardwareStatus(&connection_up, error_message, &calibration_needed, 
                &calibration_in_progress, motor_names, motor_types, temperatures, voltages, hw_errors);

        if (connection_up && !last_connection_up_flag) {
            learning_mode_on = true;
            comm->activateLearningMode(learning_mode_on);
            
            // publish one time
            std_msgs::Bool msg;
            msg.data = learning_mode_on;
            learning_mode_publisher.publish(msg);
        }
        last_connection_up_flag = connection_up;

        niryo_one_msgs::HardwareStatus msg;
        msg.header.stamp = ros::Time::now();
        msg.rpi_temperature = rpi_diagnostics->getRpiCpuTemperature();
        msg.hardware_version = hardware_version;
        msg.connection_up = connection_up;
        msg.error_message = error_message;
        msg.calibration_needed = calibration_needed;
        msg.calibration_in_progress = calibration_in_progress;
        msg.motor_names = motor_names;
        msg.motor_types = motor_types;
        msg.temperatures = temperatures;
        msg.voltages = voltages;
        msg.hardware_errors = hw_errors;

        hardware_status_publisher.publish(msg);

        publish_hardware_status_rate.sleep();
    }
}

void RosInterface::publishSoftwareVersion()
{
    double publish_software_version_frequency;
    ros::param::get("~publish_software_version_frequency", publish_software_version_frequency);
    ros::Rate publish_software_version_rate = ros::Rate(publish_software_version_frequency);

    while (ros::ok()) {
        std::vector<std::string> motor_names;
        std::vector<std::string> firmware_versions;
        comm->getFirmwareVersions(motor_names, firmware_versions);

        niryo_one_msgs::SoftwareVersion msg;
        msg.motor_names = motor_names;
        msg.stepper_firmware_versions = firmware_versions;
        msg.rpi_image_version = rpi_image_version;
        msg.ros_niryo_one_version = ros_niryo_one_version;
       
        software_version_publisher.publish(msg);
        publish_software_version_rate.sleep();
    }
}

void RosInterface::publishLearningMode()
{
    double publish_learning_mode_frequency;
    ros::param::get("~publish_learning_mode_frequency", publish_learning_mode_frequency);
    ros::Rate publish_learning_mode_rate = ros::Rate(publish_learning_mode_frequency);

    while (ros::ok()) {
        std_msgs::Bool msg;
        msg.data = learning_mode_on;
        learning_mode_publisher.publish(msg);
        publish_learning_mode_rate.sleep();
    }
}

void RosInterface::publishConveyor1Feedback()
{   
    double publish_conveyor_feedback_frequency = 2.0;
    ros::Rate publish_conveyor_feedback_rate = ros::Rate(publish_conveyor_feedback_frequency);

    while (ros::ok()) {
        niryo_one_msgs::ConveyorFeedback msg;
        bool connection_state;
        bool running;
        int16_t speed;
        int8_t direction; 
        comm->getConveyorFeedBack (6, &connection_state, &running, &speed, &direction); 
        msg.conveyor_id  = 6;
        msg.connection_state = connection_state; 
        msg.running = running;
        msg.speed = speed; 
        msg.direction = direction;  

        conveyor_1_feedback_publisher.publish(msg);
        publish_conveyor_feedback_rate.sleep();
    }
}

void RosInterface::publishConveyor2Feedback()
{   
    double publish_conveyor_feedback_frequency = 2.0;
    ros::Rate publish_conveyor_feedback_rate = ros::Rate(publish_conveyor_feedback_frequency);

    while (ros::ok()) {
        niryo_one_msgs::ConveyorFeedback msg;
        bool connection_state;
        bool running;
        int16_t speed;
        int8_t direction; 
        comm->getConveyorFeedBack (7, &connection_state, &running, &speed, &direction); 
        msg.conveyor_id  = 7;
        msg.connection_state = connection_state; 
        msg.running = running;
        msg.speed = speed; 
        msg.direction = direction;  

        conveyor_2_feedback_publisher.publish(msg);
        publish_conveyor_feedback_rate.sleep();
    }
}

void RosInterface::startPublishers()
{
    hardware_status_publisher = nh_.advertise<niryo_one_msgs::HardwareStatus>("niryo_one/hardware_status", 10);
    publish_hardware_status_thread.reset(new std::thread(boost::bind(&RosInterface::publishHardwareStatus, this))); 

    software_version_publisher = nh_.advertise<niryo_one_msgs::SoftwareVersion>("niryo_one/software_version", 10);
    publish_software_version_thread.reset(new std::thread(boost::bind(&RosInterface::publishSoftwareVersion, this)));

    learning_mode_publisher = nh_.advertise<std_msgs::Bool>("niryo_one/learning_mode", 10);
    publish_learning_mode_thread.reset(new std::thread(boost::bind(&RosInterface::publishLearningMode, this)));
    
    conveyor_1_feedback_publisher = nh_.advertise<niryo_one_msgs::ConveyorFeedback>("niryo_one/kits/conveyor_1_feedback", 10);
    publish_conveyor_1_feedback_thread.reset(new std::thread(boost::bind(&RosInterface::publishConveyor1Feedback, this)));

    conveyor_2_feedback_publisher = nh_.advertise<niryo_one_msgs::ConveyorFeedback>("niryo_one/kits/conveyor_2_feedback", 10);
    publish_conveyor_2_feedback_thread.reset(new std::thread(boost::bind(&RosInterface::publishConveyor2Feedback, this)));  
}



