cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  moveit_msgs
)

add_message_files(
  FILES
  RPY.msg
  ShiftPose.msg
  RobotState.msg
  RobotMoveCommand.msg
  ToolCommand.msg
  ProcessState.msg
  HardwareStatus.msg
  LogStatus.msg
  DigitalIOState.msg
  SoftwareVersion.msg
  Sequence.msg
  SequenceAutorunStatus.msg
  TrajectoryPlan.msg
  MatlabMoveResult.msg
  Position.msg
  Trajectory.msg
  ObjectPose.msg
  ConveyorFeedback.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  SetInt.srv
  RobotMove.srv
  ManageProcess.srv
  SetLeds.srv
  GetInt.srv
  PingDxlTool.srv
  OpenGripper.srv
  CloseGripper.srv
  PullAirVacuumPump.srv
  PushAirVacuumPump.srv
  SetDigitalIO.srv
  GetDigitalIO.srv
  GetSequenceList.srv
  ManageSequence.srv
  SetSequenceAutorun.srv
  ManagePosition.srv
  GetPositionList.srv
  ManageTrajectory.srv
  GetTrajectoryList.srv
  ChangeHardwareVersion.srv
  SendCustomDxlValue.srv
  ChangeMotorConfig.srv
  SetString.srv
  SetCalibrationCam.srv
  GetCalibrationCam.srv
  ObjDetection.srv
  EditGrip.srv
  EditWorkspace.srv
  GetTargetPose.srv
  GetWorkspaceRatio.srv
  GetWorkspaceList.srv
  TakePicture.srv
  SetConveyor.srv
  ControlConveyor.srv
  UpdateConveyorId.srv
  GetWorkspaceRobotPoses.srv
  DebugMarkers.srv
  DebugColorDetection.srv
)

add_action_files(
  DIRECTORY action
  FILES
  JoystickJoints.action
  RobotMove.action
  Tool.action
  Sequence.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib_msgs geometry_msgs moveit_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
