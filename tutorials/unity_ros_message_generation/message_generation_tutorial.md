# Generate C# code from ROS Message, Service, or Action

This example demonstrates how to generate the C# code for a ROS message using the `MessageGeneration` scripts but the same steps can be used to generate code for a ROS Service.

- In a Unity project 2020.2+ create a new directory under `Assets` name `Plugins`
- Download or clone the latest [TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) repo to your local machine and copy the `MessageGeneraton` directory to the newly created `Plugins` directory
- After the scripts load there should be a new option in the menu bar:
  ![menu](images/RosMessageGeneration_menu.png)

- Select `RosMessageGeneration` -> `Auto Generate Messages` and choose one of the options
	- Single Message
	- Package Message
	- All Messages in directory

- Change the input path to the desired msg file, ROS package directory, or directory and click `GENERATE!`
- The generated file will be saved in the default directory `Assets/RosMessages/`