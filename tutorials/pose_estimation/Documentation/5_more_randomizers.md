# Exercises for the Reader

In the main tutorial, we randomized the position and rotation of the cube. However, the perception package supports much more sophisticated environment randomization. In this (optional) section we will create a richer and more varied environment by adding one more randomizer to our scene.

In addition to the `YRotationRandomizer` and the `RobotArmObjectPositionRandomizer`, we have designed one more randomizer: 
* `UniformPoseRandomizer` - Randomizes object's position and rotation relative to a fixed starting pose, over the specified range. We will apply this to the camera, to make our trained model more robust to small inaccuracies in placing the real camera.

### Randomizing the Camera Pose

* **Action**: Select the `Simulation Scenario` GameObject and in the _**Inspector**_ tab, on the `Fixed Length Scenario` component, click on `Add Randomizer` and start typing `UniformPoseRandomizer`. For the `Random` parameter, set the min value of the Range to `-1`. We do that because we want the change of the position and the rotation in both directions for a given axis. The Randomizer's UI snippet should look like the following: 

<p align="center">
<img src="Images/5_uniform_pose_randomizer_settings.png" height=270/>
</p>

* **Action**: Now we need to add the RandomizerTag to the Camera but you can add it the GameObject you want. Select the `Main Camera` GameObject and in the _**Inspector**_ tab, click on the _**Add Component**_ button. Start typing `UniformPoseRandomizerTag` in the search bar that appears, until the `UniformPoseRandomizerTag` script is found, with a **#** icon to the left. Then double click on it.

If you press play, you should see the cube moving around the robot and rotate, the color and the intensity of the light changing but also the camera moving.

<p align="center">
<img src="Gifs/5_camera_randomizer.gif" height=600/>
</p>


## User Project: Create Your Own

You have now learned how to create a randomizer, and seen how multiple randomizers can be used together to create a rich, varied scene. Now it is time to create your own by yourself! How could this scene be further improved?

Good luck and have fun! 
