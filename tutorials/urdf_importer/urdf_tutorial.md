# Importing a Niryo One Robot using URDF Importer

## Requirements
- Unity Version 2020.2+
- [URDF Importer repo](https://github.com/Unity-Technologies/URDF-Importer)
- Niryo One URDF files from [Niryo One ROS](https://github.com/NiryoRobotics/niryo_one_ros)
- Working ROS environment

## Setting up the URDF Importer in Unity Editor
- Integrate the URDF Importer following these [instructions](https://github.com/Unity-Technologies/URDF-Importer#integrate-urdf-importer-into-unity-project)
- Create a new directory in `Assets` and name it `URDF`
- Clone the [Niryo One ROS](https://github.com/NiryoRobotics/niryo_one_ros) repo and copy the `niryo_one_description` directory into `Assets/URDF`
- This directory only includes a `.urdf.xacro` file which will need to be converted into a `.urdf` file before we can import it
	- Run the following command  to convert Xacro to URDF, `rosrun xacro xacro --inorder -o PATH/TO/niryo_one.urdf PATH/TO/niryo_one.urdf.xacro`
	- Copy the generated `niryo_one.urdf` file to `Assets/URDF`
	- Right click on the this file and select `Import Robot from URDF`
	- Select the co-ordinate system in which the meshes were designed. Default mesh orientation is Y-up which is supported by Unity but some packages often use Z-up and X-up configuration. For more [information](https://docs.unity3d.com/Manual/HOWTO-FixZAxisIsUp.html).
	- Select the Convex Mesh Decomposer you want to use for the imported robot. More information can be found [here](urdf_appendix.md##Convex-Mesh-Collider).
	- Click `Import`

## Using the Controller
- To add the controller to an imported robot click the `Enable` button in the Inspector window in front of the `Controller Script` option. This will add a Controller Script, FKrobot and Joint Control at runtime.
- To prevent the joints from slipping set the `Stiffness` and `Damping` to `100,000` and `10,000` respectively.
- To be able to apply forces to the joints set the `Force Limit` to `10,000`.
- To prevent the robot from falling over, in the GameObject tree expand `niryo_one` -> `world` -> `base_link` and set the toggle for `Immovable` for the base_link.
- Press the play button to start the scene.
	- Use the left and right arrow keys to select the articulation body you want to move.
	- Use the up and down keys to move the articulation body clockwise and counterclockwise.


A guide to making a custom controller can be found [here](urdf_appendix.md##Guide-to-write-your-own-controller)

## Forward Kinematics script

Forward Kinematics scripts gives you the ability to see the current position of the end effector based on forward kinematics of the robot. You can [use](urdf_appendix.md##Using-FK-Robot-Script) it to compare the end effector position of the robot of the articulation body to make sure the importer is working correctly.
