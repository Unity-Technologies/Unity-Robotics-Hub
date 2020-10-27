#!/usr/bin/env python

# trajectory_manager.py 
# Copyright (C) 2018 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import rospy
from niryo_one_commander.niryo_one_file_exception import NiryoOneFileException
from niryo_one_commander.trajectory.trajectory import Trajectory
from niryo_one_commander.trajectory.trajectory_command_type import TrajectoryCommandType
from niryo_one_commander.trajectory.trajectory_file_handler import TrajectoryFileHandler
from niryo_one_commander.robot_commander_exception import RobotCommanderException
from niryo_one_commander.parameters_validation import ParametersValidation

from niryo_one_msgs.msg import Trajectory
from niryo_one_msgs.srv import ManageTrajectory
from niryo_one_msgs.srv import GetTrajectoryList


class TrajectoryManager:

    def __init__(self, trajectory_dir):
        self.fh = TrajectoryFileHandler(trajectory_dir)
        self.manage_position_server = rospy.Service(
            '/niryo_one/trajectory/manage_trajectory', ManageTrajectory, self.callback_manage_trajectory)
        rospy.loginfo("/niryo_one/trajectory/manage_trajectory service has been created ")
        self.get_trajectory_list_server = rospy.Service(
            '/niryo_one/trajectory/get_trajectory_list', GetTrajectoryList, self.callback_get_trajectory_list)
        rospy.loginfo("/niryo_one/trajectory/get_trajectory_list")
        self.validation = rospy.get_param("/niryo_one/robot_command_validation")
        self.parameters_validation = ParametersValidation(self.validation)

    def callback_get_trajectory_list(self, req=None):
        traj_list = self.get_all_trajectories()
        msg_list = []
        for traj in traj_list:
            trajectory_msg = Trajectory()
            trajectory_msg.description = traj.description
            trajectory_msg.name = traj.name
            trajectory_msg.id = traj.id
            trajectory_msg.trajectory_plan = traj.trajectory_plan
            msg_list.append(trajectory_msg)
        return {'trajectories': msg_list}

    def get_all_trajectories(self):
        filenames = self.fh.get_all_filenames()
        trajectory_list = []
        for f in filenames:
            try:
                trajectory_id = self.fh.trajectory_id_from_filename(f)
                traj = self.get_trajectory(trajectory_id)
                if traj is not None:
                    trajectory_list.append(traj)
            except NiryoOneFileException as e:
                pass
        return trajectory_list

    @staticmethod
    def create_trajectory_response(status, message, trajectory=None):
        trajectory_msg = Trajectory()
        if trajectory is not None:
            trajectory_msg.trajectory_plan = trajectory.trajectory_plan
            trajectory_msg.id = trajectory.id
            trajectory_msg.name = trajectory.name
            trajectory_msg.description = trajectory.description
            return {'status': status, 'message': message, 'trajectory': trajectory_msg}
        return {'status': status, 'message': message}

    def callback_manage_trajectory(self, req):
        cmd_type = req.cmd_type
        trajectory_id = req.trajectory_id
        trajectory_data = Trajectory(name=req.trajectory.name, description=req.trajectory.description,
                                     trajectory_plan=req.trajectory.trajectory_plan)
        # GET an existing trajectory
        if cmd_type == TrajectoryCommandType.GET:
            traj = self.get_trajectory(trajectory_id)
            if traj is None:
                return self.create_trajectory_response(400, "No trajectory found with id : " + str(trajectory_id))
            return self.create_trajectory_response(200, "trajectory has been found", traj)
        # Create a new trajectory
        elif cmd_type == TrajectoryCommandType.CREATE:
            (new_trajectory_id, msg) = self.create_new_trajectory(trajectory_data)
            new_trajectory = self.get_trajectory(new_trajectory_id)
            if new_trajectory is None:
                return self.create_trajectory_response(400, msg)
            return self.create_trajectory_response(200, msg, new_trajectory)
        # UPDATE existing trajectory
        elif cmd_type == TrajectoryCommandType.UPDATE:
            traj = self.get_trajectory(trajectory_id)
            if traj is None:
                return self.create_trajectory_response(400, "No trajectory found with id : " + str(trajectory_id))
            success, update_msg = self.update_trajectory(traj, trajectory_data)
            if not success:
                return self.create_trajectory_response(400, update_msg + str(trajectory_id))
            return self.create_trajectory_response(200, update_msg, traj)
        # DELETE sequence
        elif cmd_type == TrajectoryCommandType.DELETE:
            success = self.delete_trajectory(trajectory_id)
            if not success:
                return self.create_trajectory_response(400,
                                                       "Could not delete trajectory with id : " + str(trajectory_id))
            return self.create_trajectory_response(200, "Trajectory has been deleted")
        # Wrong cmd_type 
        else:
            return self.create_trajectory_response(400, "Wrong command type")

    def delete_trajectory(self, tarjectory_id):
        try:
            self.fh.remove_trajectory(tarjectory_id)
        except NiryoOneFileException as e:
            return False
        return True

    def update_trajectory(self, traj, trajectory_data):
        traj.name = trajectory_data.name
        traj.description = trajectory_data.description
        traj.trajectory_plan = trajectory_data.trajectory_plan
        try:
            self.parameters_validation.validate_trajectory(traj.trajectory_plan)
        except RobotCommanderException as e:
            rospy.logwarn(str(e) + " Invalid trajectory ")
            return False, "Could not update trajectory : Invalid trajectory with id : "
        try:
            self.fh.write_trajectroy(traj)
        except NiryoOneFileException as e:
            return False, " Could not update trajectory with id : "
        return True, " Trajectory has been updated : "

    def create_new_trajectory(self, traj):
        new_id = self.fh.pick_new_id()
        traj.id = new_id
        try:
            self.parameters_validation.validate_trajectory(traj.trajectory_plan)
        except RobotCommanderException as e:
            rospy.logwarn(str(e) + " Invalid trajectory")
            return -1, "Failed to create trajectory: invalid trajectory "

        try:
            self.fh.write_trajectroy(traj)
        except NiryoOneFileException as e:
            return -1, "Failed to create trajectory "
        return new_id, "trajectory has been created : "

    def get_trajectory(self, trajectory_id):
        try:
            return self.fh.read_trajectory(trajectory_id)
        except NiryoOneFileException as e:
            return None


if __name__ == '__main__':
    pass
