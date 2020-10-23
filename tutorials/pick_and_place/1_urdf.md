# Pick and Place Tutorial [DRAFT]

This step includes downloading and installing the Unity Editor, setting up a basic Unity scene, and importing a robot using the URDF importer.

## Table of Contents
- [Pick and Place Tutorial [DRAFT]](#pick-and-place-tutorial-draft)
  - [Table of Contents](#table-of-contents)
  - [Step 1: Create Unity scene with imported URDF](#step-1-create-unity-scene-with-imported-urdf)
  
---

## Step 1: Create Unity scene with imported URDF
  
- Install [Unity Hub](https://unity3d.com/get-unity/download).
  
- Open Unity Hub and navigate to the Installs tab. Select `ADD` to install the latest version of Unity 2020.2 (2020.2.0b8 as of the latest revision).
   
![](img/1_hub.png) 

- In the Unity Hub, go to the Projects tab. Click the dropdown arrow next to `NEW`, and select the newly downloaded 2020.2 version. Create a new project using the 3D Template.
  
- Download the provided Unity assets [PLACEHOLDER](). Once Unity has opened, double click the `.unitypackage` file. In the Import Unity Package window that has opened in Unity, ensure everything is selected, and click `Import`. Once this is done, a folder titled PLACEHOLDER should be created in the Assets folder, containing Environment, Materials, Models, Prefabs, Plugins, and URDF subfolders.
  - If a window prompts you to reload the SampleScene, click Reload!
  
- Load the SampleScene if it is not already open. In the Unity Project window, navigate to `Assets/Prefabs`. Select the Table prefab, and click and drag it into the Hierarchy window. The table should appear in the Scene view with position and rotation `(0,0,0)`. Then, select and drag the Target into the Hierarchy window, as well as the TargetPlacement. They should appear to sit on the table.

![](img/1_cube.png) 

- Open the Physics Project Settings (Edit > Project Settings > Physics). Set the `Solver Type` to Temporal Gauss Seidel. This prevents erratic behavior in the joints that may be caused by the default solver.

![](img/1_physics.png)

<!-- - Find and select `Assets/Plugins/Urdf.dll` in the Project window and uncheck `Validate References` in the Inspector. -->
  
<!-- ![](img/1_dll.png)  -->

- Find and select the URDF file in the Project window (`Assets/PLACEHOLDER/URDF/niryo_one.urdf`). From the menu, click `Assets -> Import Robot from URDF`, or in the Project window, right click on the selected file and click `Import Robot from URDF`.
  
- Keep the default Y Axis type in the Import menu and click `Import URDF`.
  
> Note: Default mesh orientation is Y-up, which is supported by Unity, but some packages often use Z-up and X-up configuration.
> Note: PLACEHOLDER origin position
  ```xml
  <joint name="joint_world" type="fixed">
    <parent link="world" />
    <child link="base_link" />
    <origin xyz="0 0 0.765" rpy="0 0 0" />
  </joint>
  ```

- The UR3 should now be visible in the scene! Select the `niryo_one` object and set its position to `(0,0.765,0)` to place it on top of the table.
  
- In the Hierarchy window, click the arrow to the left of the name to expand the GameObject tree, down to `niryo_one/world/base_link`. Toggle on `Immovable` for the `base_link`.

![](img/1_base.png) 

> Note: A controller is pre-built in the Unity URDF importer to help showcase the movement of the Niryo. The Controller script is added to the imported URDF by default. This will add FKrobot and Joint Control components at runtime. 

- On the Controller script of the `niryo_one` object, set the Stiffness to `10000` and the Damping to `100`.

![](img/1_controller.png) 

- Press Play. If everything imported correctly, no errors should appear in the Console window. The robot arm should stay “mounted” to the table, and nothing should fall through the floor. 
  
> Note: Using the Controller, joints can be selected using the arrow keys. Use the left/right arrow keys to navigate through the joints, where the selected index will be highlighted in red. Use the up/down arrow keys to control the selected joint movement.

![](img/1_end.gif) 

- Learn more about the URDF importer tool [here](https://github.com/Unity-Technologies/Robotics-Tutorials/blob/master/urdf_tutorial.md).

---

## Troubleshooting

- If the robot appears loose/wiggly or is not moving with no console errors, ensure that the Stiffness and Damping values on the Controller script of the `niryo_one` object are set to `10000` and `100`, respectively.

---

<!-- ## Resources

- Niryo One?
--- -->


Proceed to [Step 2](2_ros_tcp.md).