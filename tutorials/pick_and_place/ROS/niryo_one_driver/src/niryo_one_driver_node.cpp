/*
    niryo_one_driver_node.cpp
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

#include <boost/shared_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <sstream>
#include <string>
#include <thread>

#include "niryo_one_driver/niryo_one_hardware_interface.h"
#include "niryo_one_driver/communication_base.h"
#include "niryo_one_driver/niryo_one_communication.h"
#include "niryo_one_driver/fake_communication.h"
#include "niryo_one_driver/ros_interface.h"
#include "niryo_one_driver/rpi_diagnostics.h"

#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "std_msgs/Empty.h"

class NiryoOneDriver {

    private:
    
    int hardware_version;

    boost::shared_ptr<CommunicationBase> comm;

    boost::shared_ptr<NiryoOneHardwareInterface> robot;
    boost::shared_ptr<controller_manager::ControllerManager> cm;

    boost::shared_ptr<RosInterface> ros_interface;

    boost::shared_ptr<RpiDiagnostics> rpi_diagnostics;

    boost::shared_ptr<ros::Rate> ros_control_loop_rate;

    boost::shared_ptr<std::thread> ros_control_thread;

    ros::NodeHandle nh_;

    bool flag_reset_controllers;

    ros::Subscriber reset_controller_subscriber; // workaround to compensate missed steps
    ros::Subscriber trajectory_result_subscriber;


    public:

    void rosControlLoop() 
    {
        ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time;
        
        while(ros::ok()) {
        
          robot->read();
          current_time = ros::Time::now();
          elapsed_time = ros::Duration(current_time - last_time);
          last_time = current_time;        

          if (flag_reset_controllers) {
            robot->setCommandToCurrentPosition();
            cm->update(ros::Time::now(), elapsed_time, true);
            flag_reset_controllers = false;
          }
          else {
            cm->update(ros::Time::now(), elapsed_time, false);
          }
          robot->write();
         
          ros_control_loop_rate->sleep();
        }
    }

    NiryoOneDriver() 
    {
        reset_controller_subscriber = nh_.subscribe("/niryo_one/steppers_reset_controller", 10,  &NiryoOneDriver::callbackTrajectoryGoal, this);
        
        trajectory_result_subscriber = nh_.subscribe("/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory/result",
                10, &NiryoOneDriver::callbackTrajectoryResult, this);

        ros::param::get("/niryo_one/hardware_version", hardware_version);
        
        if (hardware_version != 1 && hardware_version != 2) {
            ROS_ERROR("Incorrect hardware version, should be 1 or 2");
            return;
        }

        double ros_control_frequency;
        ros::param::get("~ros_control_loop_frequency", ros_control_frequency); 

        bool fake_communication;
        ros::param::get("~fake_communication", fake_communication);

        ROS_INFO("Starting niryo_one driver thread (frequency : %lf)", ros_control_frequency);

        if (fake_communication) {
            comm.reset(new FakeCommunication(hardware_version));
        }
        else {
            comm.reset(new NiryoOneCommunication(hardware_version));
        }
        
        int init_result = comm->init();
        if (init_result != 0) {
            return; // need to check last ROS_ERROR to get more info
        }
        
        ROS_INFO("NiryoOne communication has been successfully started");
       
        ros::Duration(0.1).sleep();
        flag_reset_controllers = true; 

        ROS_INFO("Start hardware control loop");
        comm->manageHardwareConnection();
        ros::Duration(0.5).sleep();

        ROS_INFO("Start hardware interface");
        robot.reset(new NiryoOneHardwareInterface(comm.get()));
        
        ROS_INFO("Create controller manager");        
        cm.reset(new controller_manager::ControllerManager(robot.get(), nh_));
        ros::Duration(0.1).sleep();
        
        ROS_INFO("Starting ros control thread...");
        ros_control_loop_rate.reset(new ros::Rate(ros_control_frequency));
        ros_control_thread.reset(new std::thread(boost::bind(&NiryoOneDriver::rosControlLoop, this)));

        ROS_INFO("Start Rpi Diagnostics...");
        rpi_diagnostics.reset(new RpiDiagnostics());

        ROS_INFO("Starting ROS interface...");
        bool learning_mode_activated_on_startup = true;
        ros_interface.reset(new RosInterface(comm.get(), rpi_diagnostics.get(), 
                    &flag_reset_controllers, learning_mode_activated_on_startup, hardware_version));

        // activate learning mode 
        comm->activateLearningMode(learning_mode_activated_on_startup);
    }

    /*
     * Problem : for joint_trajectory_controller, position command has no discontinuity
     * --> If the stepper motor missed some steps, we need to start at current position (given by the encoder)
     *  So current real position != current trajectory command, we need a discontinuity in controller command.
     *  We have to reset controllers to start from sensor position. 
     *  If we subscribe to trajectory /goal topic and reset when we receive a goal, it is often
     *  too late and trajectory will just be preempted.
     *
     *  So, in order to start from encoder position, we need to reset controller before we send the goal. If you
     *  send a new goal, be sure to send a message on /niryo_one_steppers_reset_controller BEFORE sending the goal.
     *
     *  This behavior is used in robot_commander node.
     *
     */
    void callbackTrajectoryGoal(const std_msgs::Empty& msg)
    {
        ROS_INFO("Received trajectory GOAL");
        robot->setCommandToCurrentPosition();  // set current command to encoder position
        cm->update(ros::Time::now(), ros::Duration(0.00), true); // reset controllers to allow a discontinuity in position command
        comm->synchronizeMotors(true);
    }

    void callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult& msg)
    {
        ROS_INFO("Received trajectory RESULT");
        comm->synchronizeMotors(false);
    }
};


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "niryo_one_driver");
  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    NiryoOneDriver nd; 

    ros::waitForShutdown();
    
    ROS_INFO("shutdown node");
}

