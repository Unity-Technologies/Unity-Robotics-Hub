# Niryo One description package

This package contains URDF file and meshes (collada + stl) for Niryo One.

**Note** : 3D visualization is not available on the Niryo One Raspberry Pi image. To use the following commands, you must have installed ROS on your computer and downloaded the Niryo One ROS packages. 

* To display Niryo One on Rviz :

```
roslaunch niryo_one_description display.launch
```

Also, as you can see there is a file named "without\_mesh\_niryo\_one.urdf". This file is used on the Raspberry Pi 3 to avoid a segmentation fault when using visuals, due to some ARM dependencies of Moveit!.
