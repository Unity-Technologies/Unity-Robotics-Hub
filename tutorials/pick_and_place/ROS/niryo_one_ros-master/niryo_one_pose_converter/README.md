# Niryo One Pose Converter
This package allows the use of object poses and converting them to robot poses. This package has been
developed in parallel to the vision package to allow the convertion of workspace-relative poses from
the camera to robot poses.

The main functionalities are:
- creation of workspaces
- creation of grips
- convertion of workspace-relative poses (e.g. from camera) to robot poses

## Working principle
#### Workspace
A workspace is defined by 4 markers that form a rectangle. With the help of the robot's calibration
tip, the marker positions are learnt. The camera returns poses (x, y, yaw) relative to the workspace.
We can then infer the absolute object pose in robot coordinates.

#### Grip
When we know the object pose in robot coordinates, we can't directly send this pose to the robot because
we specify the target pose of the tool_link and not of the actual TCP (tool center point). Therefore
we introduce the notion of grip. Each end effector has its own grip that specifies where to place the
robot with respect to the object. Currently, the notion of grip is not part of the python/tcp/blockly
interface because it would add an extra layer of complexity that is not really necessary for the moment.
Therefore we have a default grip for all tools that is selected automatically based on the current
tool id. However, everything is ready if you want to define custom grips, e.g. for custom tools or
for custom grip positions.

#### The whole loop
1. Camera detects object relative to markers and sends x<sub>rel</sub>, y<sub>rel</sub>, yaw<sub>rel</sub>
2. The object is placed on the workspace, revealing the object pose in robot coordinates x, y, z, roll, pitch, yaw
3. The grip is applied on the absolute object pose and gives the pose the robot should move to.
