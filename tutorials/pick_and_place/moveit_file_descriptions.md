# MoveIt Launch & Config Files

 In an attempt to make this tutorial a little more comprehensive and shed some light as to the different components of MoveIt, we have provided a minimal amount of files taken from the [Niryo ROS Stack](https://github.com/NiryoRobotics/niryo_one_ros) repo to successfully complete a pick-and-place task.

**NOTE:** The flow of which launch files call other launch files was not changed so that the workflow here would still resemble that of the original Niryo One repo even though some of the lines and arguments have been removed or changed.

### MoveIt Launch Files

**demo.launch**

Single launch file for all MoveIt and Niryo requirements for the pick-and-place tutorial.

- Loads `niryo_one_base.launch` while passing in `urdf_without_meshes` argument set to false
- Loads `controllers.launch` with `simulation_mode` set to true
- Loads `move_group.launch`
- Set rosparam `move_group/trajectory_execution/execution_duration_monitoring` to false
- Set rosparam `/move_group/trajectory_execution/allowed_start_tolerance` to 0.0
- Set rosparam `/move_group/start_state_max_bounds_error` to 0.3

**controllers.launch**

Starts the Niryo One driver, controller manager, and robot state publisher nodes

- Starts ROS node `niryo_one_driver` with debugging arguments
	- Loads `niryo_one_driver.yaml` into rosparam
- Loads `niryo_one_controllers.yaml` into rosparam
- Starts ROS node `controller_manager` with arguments `joint_state_controller niryo_one_follow_joint_trajectory_controller
        --shutdown-timeout 1`

- Starts ROS node `robot_state_publisher` package.
	- Ex. `rosrun robot_state_publisher robot_state_publisher`

**move_group.launch**

Starts the Move Group node which is the entry point into using MoveIt. Also calls planning pipeline, planning context, and trajectory execution to load their relevant parameters.

- Call `planning_pipeline.launch.xml` with `pipeline` argument set to `ompl`

- Call `trajectory_execution.launch.xml` with arguments:
  - `moveit_manage_controllers` set to true
  - `moveit_controller_manager` set to `niryo_one`

- Call `planning_context.launch`
- Start ROS node `moveit_ros_move_group`
  - Set rosparam `allow_trajectory_execution` to true
  - Set rosparam `max_safe_path_cost` to 1
  - Set rosparam `jiggle_fraction` to 0.05
  - Set rosparam`planning_scene_monitor/publish_planning_scene` to true
  - Set rosparam`planning_scene_monitor/publish_geometry_updates` to true
  - Set rosparam`planning_scene_monitor/publish_state_updates` to true
  - Set rosparam`planning_scene_monitor/publish_transforms_updates` to true

**niryo_one_base.launch**

Loads the Niryo One URDF with accompanying parameters for command validation, service timeouts, and motor groupings

- Sets rosparams `hardware_version` to 2 and `reboot_when_auto_change_version`. to true for `niryo_one` namespace

- Loads `robot_command_validation.yaml` into rosparams for namespace `niryo_one/robot_command_validation`

- Load the `niryo_one_motors.yaml` and `stepper_params.yaml` files as rosparams for `niryo_one/motors` namespace

- Sets rosparams `service_timeout` to 2, `action_connection_timeout` to 2, `action_execute_timeout` to 15, and `action_preempt_timeout` to 3 for `niryo_one/python_api` namespace

- Sets rosparams `rate_tf_listener` to 5.0 and `rate_publish_state` to 5.0 for `niryo_one/robot_state` namespace

- Loads `niryo_one_urdf.xacro` as rosparams `robot_description` and `robot_description_tf2`

**niryo_one_moveit_controller_manager.launch.xml**

Set controllers available to MoveIt

- Sets rosparam `moveit_controller_manager` to `moveit_simple_controller_manager/MoveItSimpleControllerManager`

- Reads `controllers.yaml` into rosparam

**ompl_planning_pipeline.launch.xml**

Sets ROS params and topics for OMPL pipeline communication

- Set rosparam `planning_plugin` with value of `ompl_interface/OMPLPlanner`
- Set rosparam `planning_adapters` with value of:

	```
	default_planner_request_adapters/AddTimeParameterization
					       default_planner_request_adapters/FixWorkspaceBounds
					       default_planner_request_adapters/FixStartStateBounds
					       default_planner_request_adapters/FixStartStateCollision
					       default_planner_request_adapters/FixStartStatePathConstraints
	```
- Set rosparam `start_state_max_bounds_error` with value of 0.1

- Load `ompl_planning.yaml` as rosparam

**planning_context.launch**

Loads config files required for the joints of the Niryo One.

- Argument `load_robot_description` defaults to false
- Load `kinematics.yaml` as rosparam with namespace `robot_description_kinematics`
- Load `joint_limits.yaml` as rosparam with namespace `robot_description_planning`
- Load `niryo_one.srdf` as `robot_description_semantic` rosparam
- Load `niryo_one.urdf.xacro` as `robot_description` rosparam if argument `load_robot_description` is true

**planning_pipeline.launch.xml**

Calls for OMPL pipeline launch

- Argument `pipeline` defaults to `ompl`
- Launch `ompl_planning_pipeline.launch.xml` using `pipeline` argument in name

**trajectory_execution.launch.xml**

Sets ROS params required for trajectory execution and then calls controller manager launch

- Sets rosparam `moveit_manage_controllers` to true
- Sets rosparam `trajectory_execution/allowed_execution_duration_scaling` to 1.2
- Sets rosparam `trajectory_execution/allowed_goal_duration_margin` to 0.5
- Sets rosparam `trajectory_execution/allowed_start_tolerance` to 0.0
- Loads `niryo_moveit_controller_manager.launch.xml`


### MoveIt Config Files

**controllers.yaml**

- List of robot controllers available to MoveIt.

**joint_limits.yaml**

- Velocity and acceleration limits for individual joints on robot used when planning a trajectory that is time constrained.

**kinematics.yaml**

- File generated by MoveIt Setup Assistant and contains parameters for kinematic solvers in MoveIt.
- [MoveIt Documentation](https://ros-planning.github.io/moveit_tutorials/doc/kinematics_configuration/kinematics_configuration_tutorial.html)

**niryo_one.srdf**

- Defines joint groups, group states, and disables collisions between joints and links that are impossible to collide.
- [MoveIt SRDF Documentation](https://ros-planning.github.io/moveit_tutorials/doc/urdf_srdf/urdf_srdf_tutorial.html#srdf)

**niryo_one_controllers.yaml**

- Ties `niryo_one_follow_joint_trajectory_controller` to the `controller_manager` node

**niryo_one_driver.yaml**

- Settings for communication with Niryo One motors
	- Ex. Read/Write/Check frequencies

**niryo_one_motors.yaml**

- Groups the different Niryo One motors

**ompl_planning.yaml**

- Settings required for OMPL trajectory planning.
- [MoveIt OMPL Documentation](https://ros-planning.github.io/moveit_tutorials/doc/ompl_interface/ompl_interface_tutorial.html#ompl-planner)

**robot_command_validation.yaml**

- Settings for minimum and maximum joint rotations as wel as gripper speed

**stepper_params.yaml**

- Settings for stepper motors like gear ratios and direction values.


