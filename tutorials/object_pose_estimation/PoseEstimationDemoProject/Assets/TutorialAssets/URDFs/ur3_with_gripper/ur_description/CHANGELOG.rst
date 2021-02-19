^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.7 (2019-11-23)
------------------

1.2.6 (2019-11-19)
------------------
* Add optional safety_controller tags to all joints in xacro macros (`#437 <https://github.com/ros-industrial/universal_robot/issues/437>`_)
* Migrated all package.xml files to format=2 (`#439 <https://github.com/ros-industrial/universal_robot/issues/439>`_)
* Corrected dimensions and positions of inertias (`#426 <https://github.com/ros-industrial/universal_robot/issues/426>`_)
* Add description view launch files for all descriptions to easily check them (`#435 <https://github.com/ros-industrial/universal_robot/issues/435>`_)
* Contributors: Felix Mauch, JeremyZoss, Miguel Prada, Qiang Qiu, gavanderhoorn

1.2.5 (2019-04-05)
------------------
* Add transmission_hw_interface to UR xacro and expose everywhere (`#392 <https://github.com/ros-industrial/universal_robot/issues/392>`_)
* Update maintainer listing: add Miguel (`#410 <https://github.com/ros-industrial/universal_robot/issues/410>`_)
* Updated xacro namespace.
* Update maintainer and author information.
* Updated mesh ambience so the model isn't so dark in Gazebo
* Fix overlapping variable names between robot definition files (`#356 <https://github.com/ros-industrial/universal_robot/issues/356>`_)
* Improve meshes shading (`#233 <https://github.com/ros-industrial/universal_robot/issues/233>`_)
* Added run_depend for xacro
* Using the 'doc' attribute on 'arg' elements.
* Enable self collision in gazebo
* Contributors: Dave Niewinski, Felix von Drigalski, Harsh Deshpande, Joe, Marcel Schnirring, Miguel Prada, MonteroJJ, ipa-fxm

1.2.1 (2018-01-06)
------------------
* Merge pull request `#329 <https://github.com//ros-industrial/universal_robot/issues/329>`_ from tecnalia-medical-robotics/joint_limits
  Homogenize xacro macro arguments.
* Merge pull request `#332 <https://github.com//ros-industrial/universal_robot/issues/332>`_ from davetcoleman/kinetic_hw_iface_warning
  Remove UR3 ROS Control Hardware Interface warning
* Remove UR3 ROS Control Hardware Interface warning
* Extend changes to '_robot.urdf.xacro' variants as well.
* Homogenize xacro macro arguments.
  Joint limits for the limited version could be set using arguments for the UR10
  but not for the UR3 and UR5. Same lower and upper limit arguments are added to
  the UR3 and UR5 xacro macros.
* Fix elbow joint limits (`#268 <https://github.com//ros-industrial/universal_robot/issues/268>`_)
* Remove warning 'redefining global property: pi' (Jade+) (`#315 <https://github.com//ros-industrial/universal_robot/issues/315>`_)
* Contributors: Beatriz Leon, Dave Coleman, Felix Messmer, Miguel Prada

1.2.0 (2017-08-04)
------------------

1.1.9 (2017-01-02)
------------------
* reintroduce 'pi', unbrake dependent xacros.
* use '--inorder' to trigger use of jade+ xacro on Indigo.
* Contributors: gavanderhoorn

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* Fix xacro warnings in Jade (`#251 <https://github.com/ros-industrial/universal_robot/issues/251>`_)
* added default values to xacro macro
* tested joint limits modification
* Contributors: Dave Coleman, G.A. vd. Hoorn, philip 14.04

1.1.6 (2016-04-01)
------------------
* unify mesh names
* add color to avoid default color 'red' for collision meshes
* use correct DH parameter + colored meshes
* introducing urdf for ur3 - first draft
* unify common xacro files
* remove obsolete urdf files
* description: add '_joint' suffix to newly introduced joint tags.
  This is more in-line with naming of existing joint tags.
* description: add ROS-I base and tool0 frames. Fix `#49 <https://github.com/ros-industrial/universal_robot/issues/49>`_ and `#95 <https://github.com/ros-industrial/universal_robot/issues/95>`_.
  Note that 'base' is essentially 'base_link' but rotated by 180
  degrees over the Z-axis. This is necessary as the visual and
  collision geometries appear to also have their origins rotated
  180 degrees wrt the real robot.
  'tool0' is similar to 'ee_link', but with its orientation such
  that it coincides with an all-zeros TCP setting on the UR
  controller. Users are expected to attach their own TCP frames
  to this frame, instead of updating it (see also [1]).
  [1] http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages#Standardised_links\_.2BAC8_frames
* description: minor whitespace cleanup of UR5 & 10 xacros.
* regenerate urdf files
* use PositionJointInterface as hardwareInterface in transmissions - affects simulation only
* Contributors: gavanderhoorn, ipa-fxm

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------
* changes due to file renaming
* generate urdfs from latest xacros
* file renaming
* adapt launch files in order to be able to use normal/limited xacro
* fixed typo in limits
* add joint_limited urdf.xacros for both robots
* (re-)add ee_link for both robots
* updates for latest gazebo under hydro
* remove ee_link - as in ur10
* use same xacro params as ur10
* use new transmission interfaces
* update xml namespaces for hydro
* remove obsolete urdf file
* remove obsolete urdf file
* Contributors: ipa-fxm

* Update ur10.urdf.xacro
  Corrected UR10's urdf to faithfully represent joint effort thresholds, velocity limits, and dynamics parameters.
* Update ur5.urdf.xacro
  Corrected effort thresholds and friction values for UR5 urdf.
* added corrected mesh file
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Corrected warning on xacro-files in hydro.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* fixed name of ur5 transmissions
* patched gazebo.urdf.xacro to be compatible with gazebo 1.5
* fixed copy&paste error (?)
* prefix versions of gazebo and transmission macros
* Added joint limited urdf and associated moveit package.  The joint limited package is friendlier to the default KLD IK solution
* Added ur5 moveit library.  The Kinematics used by the ur5 move it library is unreliable and should be replaced with the ur_kinematics
* Updated urdf files use collision/visual models.
* Reorganized meshes to include both collision and visual messhes (like other ROS-I robots).  Modified urdf xacro to include new models.  Removed extra robot pedestal link from urdf (urdfs should only include the robot itself).
* minor changes on ur5 xacro files
* Removed extra stl files and fixed indentions
* Renamed packages and new groovy version
* Added ur10 and renamed packages
* Contributors: Denis Štogl, IPR-SR2, Kelsey, Mathias Lüdtke, Shaun Edwards, ipa-nhg, jrgnicho, kphawkins, robot
