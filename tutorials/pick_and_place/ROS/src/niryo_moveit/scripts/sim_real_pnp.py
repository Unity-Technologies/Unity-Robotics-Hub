#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from niryo_one_msgs.msg import RobotMoveActionGoal, ToolCommand, TrajectoryPlan
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from niryo_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Gripper parameters
OPEN_COMMAND = 1
CLOSE_COMMAND = 2
GRIPPER_SPEED = 300

# IDs used to determine which values in a RobotMoveActionGoal message to execute
TOOL_ID = 11
TOOL_COMMAND_ID = 6
TRAJECTORY_COMMAND_ID = 7

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan

"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)

    if not move_group.plan():
        exception_str = """
            Trajectory could not be planned for a destination of {}\n with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, start_joint_angles)
        raise Exception(exception_str)

    plan = planCompat(move_group.plan())

    trajectory_plan = TrajectoryPlan()
    trajectory_plan.trajectory = plan

    return trajectory_plan

"""
    Create the RobotMoveActionGoal message with trajectory specific parameters.
"""
def publish_trajectory(publisher, trajectory):
    action_goal = RobotMoveActionGoal()
    action_goal.goal.cmd.Trajectory = trajectory
    action_goal.goal.cmd.cmd_type = TRAJECTORY_COMMAND_ID
    publisher.publish(action_goal)

"""
    Create the RobotMoveActionGoal message with gripper specific parameters.
"""
def publish_tool_command(publisher, gripper_command):
    tool_command = ToolCommand()
    tool_command.tool_id = TOOL_ID
    tool_command.cmd_type = gripper_command
    tool_command.gripper_open_speed = GRIPPER_SPEED
    tool_command.gripper_close_speed = GRIPPER_SPEED

    action_goal = RobotMoveActionGoal()
    action_goal.goal.cmd.tool_cmd = tool_command
    action_goal.goal.cmd.cmd_type = TOOL_COMMAND_ID
    publisher.publish(action_goal)


"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    print("Starting planning....")
    publisher = rospy.Publisher('/niryo_one/commander/robot_action/goal', RobotMoveActionGoal, queue_size=4)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # TODO: Make message type a list instead
    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
    ]

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)

    previous_ending_joint_angles = pre_grasp_pose.trajectory.joint_trajectory.points[-1].positions
    publish_trajectory(publisher, pre_grasp_pose)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
    previous_ending_joint_angles = grasp_pose.trajectory.joint_trajectory.points[-1].positions
    publish_trajectory(publisher, grasp_pose)
    publish_tool_command(publisher, CLOSE_COMMAND)

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    previous_ending_joint_angles = pick_up_pose.trajectory.joint_trajectory.points[-1].positions
    publish_trajectory(publisher, pick_up_pose)

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)
    publish_trajectory(publisher, place_pose)
    publish_tool_command(publisher, OPEN_COMMAND)

    move_group.clear_pose_targets()
    print("Done creating plan.")
    
def listener():
    rospy.init_node('sim_real_pnp', anonymous=True)

    rospy.Subscriber("sim_real_pnp", MoverServiceRequest, plan_pick_and_place)

    rospy.spin()

if __name__ == '__main__':
    listener()
