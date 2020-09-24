# Generate C# code from ROS Message File

Assumes Robotics-Tutorial repo has been downloaded

- Create a new Unity project 2020.2+
- Create a new directory under `Assets` name `Plugins`
- Download the latest `MessageGeneration` DLL file from [here](https://github.com/Unity-Technologies/Robotics-Tutorials/releases) and save them to the newly created `Plugins` directory
- `RosMessageGeneration` -> `AutoGenerateMessages` -> `Single Message...`
- Set the input file path to `PATH/TO/Robotics-Tutorials/robotics_demo/msg/PosRot.msg` and click `GENERATE!`
    - The generated file will be saved in the default directory `Assets/RosMessages/msg`


**TODO UPDATE!!**